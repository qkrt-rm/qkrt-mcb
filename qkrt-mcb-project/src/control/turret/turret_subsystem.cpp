#include "turret_subsystem.hpp"

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      m_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      m_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),
      m_desiredPitchVoltage(0.0f), m_desiredYawVoltage(0.0f),
      m_desiredElevation(0.0f), m_desiredYaw(0.0f),

      m_pitchPid({
          .kp = 25.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .maxICumulative = 500.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_yawPid({
          .kp = 24.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .maxICumulative = 5000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),

      m_desiredPitchRps(0.0f), m_desiredYawRps(0.0f),

      m_pitchRpsPid({
          .kp = 4000.0f,
          .ki = 140.0f,
          .kd = 0.0f,
          .maxICumulative = 50000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_yawRpsPid({
          .kp = 5000.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .maxICumulative = 700.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),

      m_aimLock(true),  
      m_sensitivity(1.0f),
      m_imu(drivers.bmi088),
      m_drivers(&drivers),
      m_ImuKalman(0.0f,1.0f),
      m_ImuLpf(tap::algorithms::filter::butterworth<2, tap::algorithms::filter::LOWPASS>(
        LPF_CUTOFF_HZ * 2.0 * std::numbers::pi,
        DT)),
      m_logger(drivers.logger)
{
    m_pitchHorizontalOffset = encoderToRad(config.pitchHorizontalOffset);
    m_yawForwardOffset = encoderToRad(config.yawForwardOffset);
}

void TurretSubsystem::initialize()
{
    m_pitchMotor.initialize();
    m_yawMotor.initialize();
}

void TurretSubsystem::refresh()
{
    /**
     * TODO: 
     * - Move some params to config 
     * - pitch angle must always be in range [-MAX_TURRET_ELEVATION, MAX_TURRET_ELEVATION]
     */

    if(m_drivers->isEmergencyStopActive()) 
    {
        m_pitchPid.reset();
        m_yawPid.reset();
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

        //Pitch Position Outer Loop
        float pitchError = getOptimalError(m_desiredElevation, currentPitch);
        float desiredPitchRpm = m_pitchPid.runControllerDerivateError(pitchError, DT);

        //Pitch Velocity Inner Loop
        float currentPitchRpm = m_pitchMotor.getEncoder()->getVelocity();  
        m_desiredPitchVoltage = m_pitchRpsPid.runControllerDerivateError(desiredPitchRpm - currentPitchRpm, DT);

        float currentYaw = getYaw();
        float currentYawRpm = m_ImuLpf.filterData(m_imu.getGz()) * -1;
        
        //Yaw Position Outer Loop
        float azimuthError = getOptimalError(m_desiredYaw, currentYaw);
        float desiredYawRpm = m_yawPid.runController(azimuthError, currentYawRpm, DT);

        //Yaw Velocity Inner Loop
        m_desiredYawVoltage = m_yawRpsPid.runController(desiredYawRpm - currentYawRpm, 0.0f, DT);

        // m_logger.printf("POS ERROR= %.3f\n", static_cast<double>(desiredYawRpm));
        // m_logger.delay(400);
        
    }
    else 
    {
        /**
         * Manual Velocity Control
         * TODO: 
         * - Filter IMU yaw feedback
         * - Expirement with IMU on Pitch
         * - TUNE params PID, Deadzone etc.
         * - CONVERT To Using rad/s not rev/min
         */
        
        float currentPitchRpm = m_pitchMotor.getEncoder()->getVelocity();

        float currentPitch = getPitch();
        float gainFF = 0.0f;

        float gravFF = gainFF * cos(currentPitch);

        float rawImu = m_imu.getGz();
        float lpfImu = m_ImuLpf.filterData(rawImu);

        float currentYawRpm = lpfImu * -1;
    
        float pitchRpmError = m_desiredPitchRps - currentPitchRpm;
        m_pitchRpsPid.runControllerDerivateError(pitchRpmError, DT);
        m_desiredPitchVoltage = m_pitchRpsPid.getOutput() + gravFF;
        
        float yawRpmError = m_desiredYawRps - currentYawRpm;
        m_yawRpsPid.runControllerDerivateError(yawRpmError, DT);
        m_desiredYawVoltage = m_yawRpsPid.getOutput();
        
        m_desiredYawVoltage = std::clamp(m_desiredYawVoltage, -MAX_TURRET_MOTOR_VOLTAGE, MAX_TURRET_MOTOR_VOLTAGE);

    }


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

}  // namespace control::turret