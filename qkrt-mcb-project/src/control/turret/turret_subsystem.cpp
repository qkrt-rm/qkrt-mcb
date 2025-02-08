#include "turret_subsystem.hpp"

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      _M_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      _M_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),

      _M_desiredElevation(0.0f), _M_desiredAzimuth(0.0f),
      _M_pitchAnglePid(2.0f, 0.5f, 0.1f, MAX_TURRET_MOTOR_RPM),
      _M_yawAnglePid  (2.0f, 0.5f, 0.1f, MAX_TURRET_MOTOR_RPM),

      _M_desiredPitchRpm(0.0f), _M_desiredYawRpm(0.0f),
      _M_pitchRpmPid(1.5f, 0.3f, 0.05f, MAX_TURRET_MOTOR_RPM),
      _M_yawRpmPid  (1.5f, 0.3f, 0.05f, MAX_TURRET_MOTOR_RPM),

      _M_manualControl(true),
      _M_sensitivity(1.0f),
      _M_yawForwardOffset(config.yawForwardOffset),
      _M_pitchHorizontalOffset(config.pitchHorizontalOffset)
{
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
     * - simply implementation of `void TurretSubsystem::setPitchRps(float pitchRps)`
     */

    if (_M_manualControl)
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
            float desiredPitchRpm = _M_pitchRpmPid.getValue();
            _M_pitchMotor.setDesiredOutput(desiredPitchRpm);
        }

        float yawRpmError = _M_desiredYawRpm - currentYawRpm;
        if (std::abs(yawRpmError) > DEAD_ZONE_RPM)
        {
            _M_yawRpmPid.update(yawRpmError);
            float desiredYawRpm = _M_yawRpmPid.getValue();
            _M_yawMotor.setDesiredOutput(desiredYawRpm);
        }
    }
    else
    {
        /**
         * Use angle PID to point turret at target elevation/azimuth.
         */
        
        float currentElevation = getElevation();
        float currentAzimuth = getAzimuth();
        
        float elevationError = _M_desiredElevation - currentElevation;
        if (std::abs(elevationError) > DEAD_ZONE_ANGLE)
        {
            _M_pitchAnglePid.update(elevationError);
            float desiredPitchAngVel = _M_pitchAnglePid.getValue();
            float desiredPitchRpm    = radPerSecToRpm(desiredPitchAngVel);
            _M_pitchMotor.setDesiredOutput(desiredPitchRpm);
        }
        
        float azimuthError = _M_desiredAzimuth - currentAzimuth;
        if (std::abs(azimuthError) > DEAD_ZONE_ANGLE)
        {
            _M_yawAnglePid.update(azimuthError);
            float desiredYawAngVel = _M_yawAnglePid.getValue();
            float desiredYawRpm    = radPerSecToRpm(desiredYawAngVel);
            _M_yawMotor.setDesiredOutput(desiredYawRpm);
        }
    }
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    if (!_M_manualControl) return;

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
    if (!_M_manualControl) return;
    
    _M_desiredYawRpm = std::clamp(
        rpsToRpm(yawRps * _M_sensitivity), 
        -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM
    );
}

}  // namespace control::turret