#include "turret_command.hpp"

TurretCommand::TurretCommand(TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface)
    : _M_turret(turret),
      _M_operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize() override
{
}

void HolonomicChassisCommand::execute()
{
    float pitch = _M_operatorInterface.getChassisPitchInput();
    float yaw = _M_operatorInterface.getChassisYawInput();

    _M_turret.adjustPitch(pitch);
    _M_turret.adjustYaw(yaw);
}

void TurretCommand::end(bool /* interrupted */)
{
}