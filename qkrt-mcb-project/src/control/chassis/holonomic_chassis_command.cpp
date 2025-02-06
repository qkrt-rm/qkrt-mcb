#include "holonomic_chassis_command.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& operatorInterface)
    : _M_chassis(chassis),
      _M_turret(turret),
      _M_operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&chassis);
}

void HolonomicChassisCommand::initialize()
{
}

void HolonomicChassisCommand::execute()
{
    float x = _M_operatorInterface.getChassisXInput();
    float z = _M_operatorInterface.getChassisZInput();
    float r = 0.0f;

    float denominator = std::max(std::abs(x) + std::abs(z) + std::abs(r), 1.0f);
    float leftFront  = (z + x + r) / denominator;
    float leftBack   = (z - x + r) / denominator;
    float rightFront = (z - x - r) / denominator;
    float rightBack  = (z + x - r) / denominator;

    _M_chassis.setWheelVelocities(leftFront, leftBack, rightFront, rightBack);
}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    _M_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
