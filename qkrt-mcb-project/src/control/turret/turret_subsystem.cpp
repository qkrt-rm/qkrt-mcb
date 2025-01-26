#include "turret_subsystem.hpp"

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers)
      _M_desiredOutput(),
      _M_pidControllers(),
      _M_motors({
          Motor(&drivers, config.pitchId,  config.canBus, false, "PITCH"),
          Motor(&drivers, config.yawId,    config.canBus, false, "YAW")
      })
{
    for (auto& controller : _M_pidControllers)
    {
        controller.setParameter(config.turretVelocityPidConfig);
    }
}

void TurretSubsystem::initialize() override
{
    for (auto& motor : _M_motors)
    {
        motor.initialize();
    }
}

void TurretSubsystem::refresh() override
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