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
    float r = _M_operatorInterface.getChassisRInput();

    float xx = (float) x * 1.1; // Counteract imperfect strafing
    float yy = (float) -z; // Y stick is reversed!
    float rr = (float) r;

    float denominator = std::max(std::abs(yy) + std::abs(xx) + std::abs(rr), 1.0f);
    float leftFront = (yy + xx + rr) / denominator;
    float leftBack = (yy - xx + rr) / denominator;
    float rightFront = (yy - xx - rr) / denominator;
    float rightBack = (yy + xx - rr) / denominator;

    _M_chassis.setWheelVelocities(leftFront, leftBack, rightFront, rightBack);
}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    _M_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
