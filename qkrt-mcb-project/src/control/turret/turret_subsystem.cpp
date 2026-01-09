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

      m_desiredPitchRpm(0.0f), m_desiredYawRpm(0.0f),

      m_pitchRpmPid({
          .kp = 400.0f,
          .ki = 14.0f,
          .kd = 0.0f,
          .maxICumulative = 5000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_yawRpmPid({
          .kp = 800.0f,
          .ki = 3.0f,
          .kd = 0.0f,
          .maxICumulative = 70.0f,
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
        float currentPitchRpm = m_pitchMotor.getEncoder()->getVelocity() * 9.55f;  //TODO:REMOVE scaling to use rps
        m_desiredPitchVoltage = m_pitchRpmPid.runControllerDerivateError(desiredPitchRpm - currentPitchRpm, DT);

        float currentYaw = getYaw();
        float currentYawRpm = m_ImuLpf.filterData(m_imu.getGz()) * -1 * 9.55f;
        
        //Yaw Position Outer Loop
        float azimuthError = getOptimalError(m_desiredYaw, currentYaw);
        float desiredYawRpm = m_yawPid.runController(azimuthError, currentYawRpm, DT);

        //Yaw Velocity Inner Loop
        m_desiredYawVoltage = m_yawRpmPid.runController(desiredYawRpm - currentYawRpm, 0.0f, DT);

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
        
        float currentPitchRpm = m_pitchMotor.getEncoder()->getVelocity() * 9.55f;  //TODO:REMOVE scaling to use rps

        float currentPitch = getPitch();
        float gainFF = 2000.0;

        float gravFF = gainFF * cos(currentPitch);

        float rawImu = m_imu.getGz();
        float lpfImu = m_ImuLpf.filterData(rawImu);

        float currentYawRpm = lpfImu * -1 * 9.55f;  
    
        float pitchRpmError = m_desiredPitchRpm - currentPitchRpm;
        m_pitchRpmPid.runControllerDerivateError(pitchRpmError, DT);
        m_desiredPitchVoltage = m_pitchRpmPid.getOutput() + gainFF;
        
        float yawRpmError = m_desiredYawRpm - currentYawRpm;
        m_yawRpmPid.runControllerDerivateError(yawRpmError, DT);
        m_desiredYawVoltage = m_yawRpmPid.getOutput();
        
        m_desiredYawVoltage = std::clamp(m_desiredYawVoltage, -MAX_TURRET_MOTOR_VOLTAGE, MAX_TURRET_MOTOR_VOLTAGE);

        // m_logger.printf("FF= %.5f\n", static_cast<double>(m_desiredPitchVoltage));
        // m_logger.delay(400);
    }


    m_pitchMotor.setDesiredOutput(m_desiredPitchVoltage);
    m_yawMotor.setDesiredOutput(m_desiredYawVoltage);
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    float pitchAngle = getPitch();
    {

        //REMOVE when switching to rad/s not rev/s
        m_desiredPitchRpm = std::clamp(
            rpsToRpm(pitchRps * m_sensitivity), 
            -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM
        );
    }
}

void TurretSubsystem::setYawRps(float yawRps)
{
    m_desiredYawRpm = std::clamp(
        rpsToRpm(yawRps * m_sensitivity), 
        -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM
    );
}

}  // namespace control::turret