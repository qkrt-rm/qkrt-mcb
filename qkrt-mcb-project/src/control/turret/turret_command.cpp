#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface,
                             Uart& uart)
    : _M_turret(turret),
      _M_operatorInterface(operatorInterface),
      _M_uart(uart),
      _M_pitchSensitivity(1.0f), _M_yawSensitivity(1.0f)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    float pitch = _M_operatorInterface.getChassisPitchInput();
    float yaw = _M_operatorInterface.getChassisYawInput();

    _M_turret.setPitchRps(pitch);
    _M_turret.setYawRps(yaw);
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret