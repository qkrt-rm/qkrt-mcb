#include "turret_subsystem.hpp"

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      m_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      m_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),
      m_desiredPitchVoltage(0.0f), m_desiredYawVoltage(0.0f),

      m_desiredElevation(0.0f), m_desiredAzimuth(0.0f),

      m_elevationPid({
          .kp = 4000.0f,
          .ki = 200.0f,
          .kd = 0.0f,
          .maxICumulative = 500.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_azimuthPid({
          .kp = 3000.0f,
          .ki = 200.0f,
          .kd = 25.0f,
          .maxICumulative = 5000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),

      m_desiredPitchRpm(0.0f), m_desiredYawRpm(0.0f),

      m_pitchRpmPid({
          .kp = 400.5f,
          .ki = 80.0f,
          .kd = 0.0f,
          .maxICumulative = 5000.0f,
          .maxOutput = MAX_TURRET_MOTOR_VOLTAGE
      }),
      m_yawRpmPid({
          .kp = 1000.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .maxICumulative = 5000.0f,
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
        m_elevationPid.reset();
        m_azimuthPid.reset();
        m_desiredPitchVoltage = 0.0f;
        m_desiredYawVoltage = 0.0f;
    }
    else if (true)      //set to false in contructor
    {
        /**
         * AUTO AIM Position Control
         * TODO:
         * - Implement Cascade Position -> Velocity Control
         */

        /**
         * @brief Computes the shortest angular error between two angles.
         *
         * This function calculates the smallest difference between a desired angle 
         * and the current angle, ensuring the result is within the range [-π, π]. 
         * This prevents issues where an angle error of, for example, 350° would be 
         * treated as -350° instead of 10°.
         *
         * @param desiredAngle The target angle in radians.
         * @param currentAngle The current angle in radians.
         * @return The shortest angular difference in radians, constrained to [-π, π].
         */
        auto getOptimalError = [](float desiredAngle, float currentAngle) -> float
            {
                float error = desiredAngle - currentAngle;
                return std::atan2(std::sin(error), std::cos(error));
            };
        
        float currentElevation = getElevation();
        float currentAzimuth = getAzimuth();
        
        float elevationError = getOptimalError(m_desiredElevation, currentElevation);

        // runControllerDerivateError calculates the derivative using (error - prev) / dt
        // m_logger.printf("Elevation Error: %.4f | DESIRED %.3f | MEASURE: %.3f \n", static_cast<double>(elevationError), static_cast<double>(m_desiredElevation), static_cast<double>(currentElevation));
        // m_logger.delay(100);

        m_desiredPitchVoltage = m_elevationPid.runControllerDerivateError(elevationError, DT);
        
        float azimuthError = getOptimalError(m_desiredAzimuth, currentAzimuth);
        
        // m_logger.printf("Elevation Error: %.4f | DESIRED %.3f | MEASURE: %.3f \n", static_cast<double>(azimuthError), static_cast<double>(m_desiredAzimuth), static_cast<double>(currentAzimuth));
        m_logger.delay(400);

        m_desiredYawVoltage = m_azimuthPid.runControllerDerivateError(azimuthError, DT);
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

        float rawImu = m_imu.getGz();
        float lpfImu = m_ImuLpf.filterData(rawImu);

        float currentYawRpm = lpfImu * -1 * 9.55f;  
    
        float pitchRpmError = m_desiredPitchRpm - currentPitchRpm;
        m_pitchRpmPid.runController(pitchRpmError, currentPitchRpm, DT);
        m_desiredPitchVoltage = m_pitchRpmPid.getOutput();
        
        float yawRpmError = m_desiredYawRpm - currentYawRpm;
        m_yawRpmPid.runController(yawRpmError, currentYawRpm, DT);
        m_desiredYawVoltage = m_yawRpmPid.getOutput();

        m_logger.printf("Elevation Error: %.4f | DESIRED %.3f | MEASURE: %.3f \n", static_cast<double>(yawRpmError), static_cast<double>(m_desiredYawRpm), static_cast<double>(currentYawRpm));
        m_logger.delay(400);
    }


    m_pitchMotor.setDesiredOutput(m_desiredPitchVoltage);
    m_yawMotor.setDesiredOutput(m_desiredYawVoltage);
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    float pitchAngle = getElevation();
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