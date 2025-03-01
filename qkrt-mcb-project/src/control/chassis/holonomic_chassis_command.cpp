#include "holonomic_chassis_command.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& operatorInterface,
                                                 tap::communication::sensors::imu::bmi088::Bmi088& imu)
    : _M_chassis(chassis),
      _M_turret(turret),
      _M_operatorInterface(operatorInterface),
      _M_imu(imu)
{
    addSubsystemRequirement(&chassis);
}

void HolonomicChassisCommand::initialize()
{
}

void HolonomicChassisCommand::execute()
{

    static bool beyblade_on = _M_operatorInterface.getWheelButton();
    
    float xInp = _M_operatorInterface.getChassisXInput();
    float zInp = _M_operatorInterface.getChassisZInput();
    float yawAngle = _M_turret.getAzimuth();

    float test = _M_imu.getPitch();
    
    
    float x = xInp * std::cos(yawAngle) - zInp * std::sin(yawAngle);
    float z = xInp * std::sin(yawAngle) + zInp * std::cos(yawAngle);
    float r = beyblade_on ? .2f : 0;
    
    float denominator = std::max(std::abs(x) + std::abs(z) + std::abs(r), 1.0f);
    float leftFront  = (z + x + r) / denominator;
    float leftBack   = (z - x + r) / denominator;
    float rightFront = (z - x - r) / denominator;
    float rightBack  = (z + x - r) / denominator;

    _M_chassis.setWheelVelocities(leftFront, leftBack, rightBack, rightFront);
    
}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    _M_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
