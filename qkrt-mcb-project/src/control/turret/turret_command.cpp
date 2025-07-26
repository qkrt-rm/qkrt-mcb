#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers& drivers, TurretSubsystem& turret)
    : _M_turret(turret),
      _M_operatorInterface(drivers.controlOperatorInterface),
      _M_logger(drivers.logger),
      _M_pitchSensitivity(1.0f), _M_yawSensitivity(1.0f),
      _M_target(nullptr)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    if (false)
    {
        _M_turret.lock();

        float desiredElevation = 0.0f;
        float desiredAzimuth = 0.0f;

        _M_turret.setElevation(desiredElevation);
        _M_turret.setAzimuth(desiredAzimuth);
    }
    else
    {
        _M_turret.unlock();

        float pitchInp = _M_operatorInterface.getTurretPitchInput();
        float yawInp = _M_operatorInterface.getTurretYawInput();

        _M_logger.printf("turret yaw (azimuth): %.3f\n", _M_turret.getAzimuth());
        _M_logger.delay(200);

        _M_turret.setPitchRps(pitchInp);
        _M_turret.setYawRps(yawInp);
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret