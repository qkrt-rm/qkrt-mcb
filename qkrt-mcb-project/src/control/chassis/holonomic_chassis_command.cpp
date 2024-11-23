#include "holonomic_chassis_command.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(HolonomicChassisSubsystem& chassis,
                                                 ControlOperatorInterface& operatorInterface)
    : _M_chassis(chassis),
      _M_operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&chassis);
}

void HolonomicChassisCommand::initialize()
{
}

void HolonomicChassisCommand::execute()
{

}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    _M_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
