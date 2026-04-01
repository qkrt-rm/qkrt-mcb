#include "turret_subsystem.hpp"

using ImuState = tap::communication::sensors::imu::ImuInterface::ImuState;

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      m_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      m_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),
      m_desiredPitchVoltage(0.0f), m_desiredYawVoltage(0.0f),
      m_desiredPitch(0.0f), m_desiredYaw(0.0f),
      m_desiredPitchRps(0.0f), m_desiredYawRps(0.0f),

      m_pitchPid({
          .kp = 15.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .maxICumulative = 500.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_yawPid({
          .kp = 5.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .maxICumulative = 5000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE,
      }),

      m_pitchRpsPid({
          .kp = 4000.0f,
          .ki = 110.0f,
          .kd = 0.0f,
          .maxICumulative = 3000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_yawRpsPid({
          .kp = 8000.0f,
          .ki = 10.0f,
          .kd = 0.0f,
          .maxICumulative = 1000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE,
          .tQProportionalKalman = 1.0f,
          .tRProportionalKalman = 0.01f, 
      }),

      m_aimLock(false),  
      m_isCalibrated(false),
      m_isChassisRot(false),
      m_sensitivity(1.0f),
      m_imu(drivers.bmi088),
      m_drivers(&drivers),
      m_logger(drivers.logger)
{
      m_pitchOffset = degToRad(config.pitchHorizontalOffset);
      m_yawOffset = degToRad(config.yawForwardOffset); 
}

void TurretSubsystem::initialize()
{
    m_pitchMotor.initialize();
    m_yawMotor.initialize();

    m_pitchPid.reset();
    m_yawPid.reset();
    m_pitchRpsPid.reset();
    m_yawRpsPid.reset();
}

void TurretSubsystem::refresh()
{
    /**
     * TODO: 
     * - Move some params to config 
     * - pitch angle must always be in range [-MAX_TURRET_ELEVATION, MAX_TURRET_ELEVATION]
     */


    if(m_drivers->isEmergencyStopActive() || m_imu.getImuState() == ImuState::IMU_CALIBRATING) 
    {
        m_pitchPid.reset();
        m_yawPid.reset();
        m_yawRpsPid.reset();
        m_pitchRpsPid.reset();

        m_desiredPitchVoltage = 0.0f;
        m_desiredYawVoltage = 0.0f;
    }
    else if (m_aimLock)     
    {
        /**
         * AUTO AIM Position Control
         * TODO:
         * - Implement Cascade Position -> Velocity Control
         */

        float currentPitch = getPitch();

        float pitchKFF = 2600.0f;
        float pitchFF = pitchKFF * cos(currentPitch);

        //Pitch Position Outer Loop
        float pitchError = getOptimalError(m_desiredPitch, currentPitch);
        float desiredPitchRpm = m_pitchPid.runControllerDerivateError(pitchError, DT);

        //Pitch Velocity Inner Loop
        float currentPitchRpm = m_pitchMotor.getEncoder()->getVelocity();  
        m_pitchRpsPid.runControllerDerivateError(desiredPitchRpm - currentPitchRpm, DT);
        m_desiredPitchVoltage = m_pitchRpsPid.getOutput();

        float currentYawRpm = m_imu.getGz() * -1;  //TODO: make sure this is filtered
        
        //Yaw Position Outer Loop
        float azimuthError = getOptimalError(m_desiredYaw, getYaw());
        float desiredYawRpm = m_yawPid.runController(azimuthError, currentYawRpm, DT);

        //Yaw Velocity Inner Loop
        m_desiredYawVoltage = m_yawRpsPid.runController(desiredYawRpm - currentYawRpm, 0.0f, DT);

        // m_logger.printf("POS ERROR= %.3f\n", static_cast<double>(desiredYawRpm));
        // m_logger.delay(400);
        
    }
    else 
    {
        //Manual Velocity PID

        float pitchKFF = 2600.0f;
        float yawKFF = 6560.0f;
    
        float yawFF = (m_isChassisRot) ? (yawKFF * -chassis::HolonomicChassisCommand::CHASSIS_ROT_SPEED_RAD) : 0.0f;
        float pitchFF = pitchKFF * cos(getPitch());

        float pitchError = getOptimalError(m_desiredPitch, getPitch());
        float desiredPitchRps = m_pitchPid.runControllerDerivateError(pitchError, DT);

        float pitchRpsError = desiredPitchRps - (m_pitchMotor.getEncoder()->getVelocity());
        m_pitchRpsPid.runControllerDerivateError(pitchRpsError, DT);
        m_desiredPitchVoltage = m_pitchRpsPid.getOutput() + pitchFF;
        
        float yawRpsError = m_desiredYawRps - (m_imu.getGz() * -1);
        m_yawRpsPid.runControllerDerivateError(yawRpsError, DT);
        m_desiredYawVoltage = m_yawRpsPid.getOutput() + yawFF;

    }

    m_desiredPitchVoltage = std::clamp(m_desiredPitchVoltage, -MAX_TURRET_MOTOR_VOLTAGE, MAX_TURRET_MOTOR_VOLTAGE);
    m_desiredYawVoltage = std::clamp(m_desiredYawVoltage, -MAX_TURRET_MOTOR_VOLTAGE, MAX_TURRET_MOTOR_VOLTAGE);

    m_pitchMotor.setDesiredOutput(m_desiredPitchVoltage);
    m_yawMotor.setDesiredOutput(m_desiredYawVoltage);
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    float pitchAngle = getPitch();
    {
        m_desiredPitchRps = std::clamp(
            pitchRps * (2.0f * static_cast<float>(M_PI)) * m_sensitivity, 
            -MAX_TURRET_MOTOR_RPS, MAX_TURRET_MOTOR_RPS
        );
    }
}

void TurretSubsystem::setYawRps(float yawRps)
{
    m_desiredYawRps = std::clamp(
        yawRps * (2.0f * static_cast<float>(M_PI)) * m_sensitivity, 
        -MAX_TURRET_MOTOR_RPS, MAX_TURRET_MOTOR_RPS
    );
}

void TurretSubsystem::ChassisRot(bool isRot) { m_isChassisRot = isRot; }

}  // namespace control::turret