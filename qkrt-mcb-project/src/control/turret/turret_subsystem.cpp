#include "turret_subsystem.hpp"

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      _M_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      _M_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),
      _M_desiredPitchVoltage(0.0f), _M_desiredYawVoltage(0.0f),

      _M_desiredElevation(0.0f), _M_desiredAzimuth(0.0f),
      _M_elevationPid(4500.0f, 10.0f, 90.0f, MAX_TURRET_MOTOR_VOLTAGE),
      _M_azimuthPid  (4500.0f, 10.0f, 120.0f, MAX_TURRET_MOTOR_VOLTAGE),

      _M_desiredPitchRpm(0.0f), _M_desiredYawRpm(0.0f),
      _M_pitchRpmPid(2.5f, 0.2f, 1.0f, MAX_TURRET_MOTOR_VOLTAGE),
      _M_yawRpmPid  (2.5f, 0.2f, 1.0f, MAX_TURRET_MOTOR_VOLTAGE),

      _M_aimLock(true),
      _M_sensitivity(1.0f)
{
    _M_pitchHorizontalOffset = encoderToRad(config.pitchHorizontalOffset);
    _M_yawForwardOffset = encoderToRad(config.yawForwardOffset);
}

void TurretSubsystem::initialize()
{
    _M_pitchMotor.initialize();
    _M_yawMotor.initialize();
}

void TurretSubsystem::refresh()
{
    /**
     * TODO: constrain pitch motor angles using PID controller
     * - adjust error to respect pitch motor constraints smoothly
     * - pitch angle must always be in range [-MAX_TURRET_ELEVATION, MAX_TURRET_ELEVATION]
     * - simplify implementation of `void TurretSubsystem::setPitchRps(float pitchRps)`
     */

    if (false)
    {
        /**
         * Use angle PID to point turret at target elevation/azimuth.
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
        
        float elevationError = getOptimalError(_M_desiredElevation, currentElevation);
        if (std::abs(elevationError) > DEAD_ZONE_ANGLE)
        {
            _M_elevationPid.update(elevationError);
            _M_desiredPitchVoltage = _M_elevationPid.getValue();
        }
        else
        {
            _M_elevationPid.reset();
            _M_desiredPitchVoltage = 0.0f;
        }
        
        float azimuthError = getOptimalError(_M_desiredAzimuth, currentAzimuth);
        if (std::abs(azimuthError) > DEAD_ZONE_ANGLE)
        {
            _M_azimuthPid.update(azimuthError);
            _M_desiredYawVoltage = _M_azimuthPid.getValue();
        }
        else
        {
            _M_azimuthPid.reset();
            _M_desiredYawVoltage = 0.0f;
        }
    }
    else
    {
        /**
         * Use RPM PID to continuously rotate turret pitch and yaw based on manual input.
         */
        
        float currentPitchRpm = _M_pitchMotor.getShaftRPM();
        float currentYawRpm = _M_yawMotor.getShaftRPM();
    
        float pitchRpmError = _M_desiredPitchRpm - currentPitchRpm;
        if (std::abs(pitchRpmError) > DEAD_ZONE_RPM)
        {
            _M_pitchRpmPid.update(pitchRpmError);
            _M_desiredPitchVoltage = _M_pitchRpmPid.getValue();
        }
    
        float yawRpmError = _M_desiredYawRpm - currentYawRpm;
        if (std::abs(yawRpmError) > DEAD_ZONE_RPM)
        {
            _M_yawRpmPid.update(yawRpmError);
            _M_desiredYawVoltage = _M_yawRpmPid.getValue();
        }
    }

    _M_pitchMotor.setDesiredOutput(_M_desiredPitchVoltage);
    _M_yawMotor.setDesiredOutput(_M_desiredYawVoltage);
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    float pitchAngle = getElevation();
    if (std::abs(pitchAngle) > MAX_TURRET_ELEVATION)
    {
        _M_desiredPitchRpm = 0.0f;
    }
    else
    {
        _M_desiredPitchRpm = std::clamp(
            rpsToRpm(pitchRps * _M_sensitivity), 
            -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM
        );
    }
}

void TurretSubsystem::setYawRps(float yawRps)
{
    _M_desiredYawRpm = std::clamp(
        rpsToRpm(yawRps * _M_sensitivity), 
        -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM
    );
}

}  // namespace control::turret