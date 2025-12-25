#include "turret_subsystem.hpp"

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      m_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      m_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),
      m_desiredPitchVoltage(0.0f), m_desiredYawVoltage(0.0f),

      m_desiredElevation(0.0f), m_desiredAzimuth(0.0f),
      m_elevationPid(4500.0f, 10.0f, 90.0f, MAX_TURRET_MOTOR_VOLTAGE),
      m_azimuthPid  (4500.0f, 10.0f, 120.0f, MAX_TURRET_MOTOR_VOLTAGE),

      m_desiredPitchRpm(0.0f), m_desiredYawRpm(0.0f),
      m_pitchRpmPid(2.5f, 0.2f, 1.0f, MAX_TURRET_MOTOR_VOLTAGE),
      m_yawRpmPid  (350.0f, 2.5f, 0.0f, MAX_TURRET_MOTOR_VOLTAGE),

      m_aimLock(false),  
      m_sensitivity(1.0f),
      m_imu(drivers.bmi088)
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
    
    if (m_aimLock)      //set to false in contructor
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
                return std::atan2(std::sin(error), std::cos(error));;
            };
        
        float currentElevation = getElevation();
        float currentAzimuth = getAzimuth();
        
        float elevationError = getOptimalError(m_desiredElevation, currentElevation);
        if (std::abs(elevationError) > DEAD_ZONE_ANGLE)
        {
            m_elevationPid.update(elevationError);
            m_desiredPitchVoltage = m_elevationPid.getValue();
        }
        else
        {
            m_elevationPid.reset();
            m_desiredPitchVoltage = 0.0f;
        }
        
        float azimuthError = getOptimalError(m_desiredAzimuth, currentAzimuth);
        if (std::abs(azimuthError) > DEAD_ZONE_ANGLE)
        {
            m_azimuthPid.update(azimuthError);
            m_desiredYawVoltage = m_azimuthPid.getValue();
        }
        else
        {
            m_azimuthPid.reset();
            m_desiredYawVoltage = 0.0f;
        }
    }
    else
    {
        /**
         * Manual Velocity Control
         * TODO: 
         * - Filter IMU yaw feedback
         * - Expirement with IMU on Pitch
         * - TUNE params PID, Deadzone etc.
         */
        
        float currentPitchRpm = m_pitchMotor.getShaftRPM();
        float currentYawRpm = dpsToRpm(m_imu.getGz()) * -1;  
    
        float pitchRpmError = m_desiredPitchRpm - currentPitchRpm;
        if (std::abs(pitchRpmError) > DEAD_ZONE_RPM)
        {
            m_pitchRpmPid.update(pitchRpmError);
            m_desiredPitchVoltage = m_pitchRpmPid.getValue();
        }
    
        float yawRpmError = m_desiredYawRpm - currentYawRpm;
        if (std::abs(yawRpmError) > DEAD_ZONE_RPM)
        {
            m_yawRpmPid.update(yawRpmError);
            m_desiredYawVoltage = m_yawRpmPid.getValue();
        }
    }

    m_pitchMotor.setDesiredOutput(m_desiredPitchVoltage);
    m_yawMotor.setDesiredOutput(m_desiredYawVoltage);
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    float pitchAngle = getElevation();
    {
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