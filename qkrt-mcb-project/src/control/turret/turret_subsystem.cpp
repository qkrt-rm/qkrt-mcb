#include "turret_subsystem.hpp"

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      _M_desiredOutput(),
      _M_pidControllers(),
      _M_motors({
          Motor(&drivers, config.pitchId,  config.canBus, false, "PITCH"),
          Motor(&drivers, config.yawId,    config.canBus, false, "YAW")
      }),
      _M_elevation(0.0f), _M_azimuth(0.0f),
      _M_sensitivity(0.1f),
      _M_yawForwardOffset(0.0f)
{
    for (auto& controller : _M_pidControllers)
    {
        controller.setParameter(config.turretVelocityPidConfig);
    }
}

void TurretSubsystem::initialize()
{
    for (auto& motor : _M_motors)
    {
        motor.initialize();
    }
}

void TurretSubsystem::refresh()
{
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput) -> void
    {
        pid.update(desiredOutput - motor.getShaftRPM());
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < _M_motors.size(); ii++)
    {
        runPid(_M_pidControllers[ii], _M_motors[ii], _M_desiredOutput[ii]);
    }
}

void TurretSubsystem::setPitch(float elevation)
{
    elevation = rpsToRpm(elevation);
    elevation = std::clamp(elevation, -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM);
    _M_desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = elevation;
}

void TurretSubsystem::setYaw(float azimuth)
{
    azimuth = rpsToRpm(azimuth);
    azimuth = std::clamp(azimuth, -MAX_TURRET_MOTOR_RPM, MAX_TURRET_MOTOR_RPM);
    _M_desiredOutput[static_cast<uint8_t>(MotorId::YAW)] = azimuth;
}

}  // namespace control::turret