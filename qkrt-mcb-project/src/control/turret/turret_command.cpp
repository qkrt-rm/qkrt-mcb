#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface,
                             Uart& uart)
    : _M_turret(turret),
      _M_operatorInterface(operatorInterface),
      _M_uart(uart),
      _M_pitchSensitivity(1.0f), _M_yawSensitivity(1.0f),
      _M_aimAssist(false)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    // if (_M_aimAssist /* && target */)
    // {
    //     float elevation = 0.0f;
    //     float azimuth = 0.0f;

    //     _M_turret.setElevation(elevation);
    //     _M_turret.setAzimuth(azimuth);
    // }
    // else
    // {
        float pitchInp = _M_operatorInterface.getTurretPitchInput();
        float yawInp = _M_operatorInterface.getTurretYawInput();
    
        _M_turret.setPitchRps(pitchInp);
        _M_turret.setYawRps(yawInp);
    // }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret