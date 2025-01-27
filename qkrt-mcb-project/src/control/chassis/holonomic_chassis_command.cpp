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
    float x = _M_operatorInterface.getChassisXInput();
    float z = _M_operatorInterface.getChassisZInput();
    float r = 0.0f;

    double denominator = std::max(std::abs(x) + std::abs(z) + std::abs(r), 1.0f);
    double leftFront  = (z + x + r) / denominator;
    double leftBack   = (z - x + r) / denominator;
    double rightFront = (z - x - r) / denominator;
    double rightBack  = (z + x - r) / denominator;

    _M_chassis.setWheelVelocities(leftFront,
                                  leftBack,
                                  rightFront,
                                  rightBack);
}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    _M_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
