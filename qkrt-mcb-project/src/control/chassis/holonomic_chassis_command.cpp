#include "holonomic_chassis_command.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& m_operatorInterface)
    : m_chassis(chassis),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface)
{
    addSubsystemRequirement(&chassis);
}

void HolonomicChassisCommand::initialize()
{
}

void HolonomicChassisCommand::execute()
{
    m_operatorInterface.pollInputDevices();

    float xInp = m_operatorInterface.getChassisXInput();
    float yInp = m_operatorInterface.getChassisYInput();
    float yawAngle = m_turret.getAzimuth();
    
    //compute rotation transformation
    float v_y = yInp * std::cos(-yawAngle) - xInp * std::sin(-yawAngle);
    float v_x = yInp * std::sin(-yawAngle) + xInp * std::cos(-yawAngle);
    float w = m_operatorInterface.getChassisWInput();       
    
    float denominator = std::max(std::abs(v_y) + std::abs(v_x) + std::abs(w), 1.0f);
    float leftFront  = (v_x + v_y + w) / denominator;
    float leftBack   = (v_x - v_y + w) / denominator;
    float rightFront = (v_x - v_y - w) / denominator;
    float rightBack  = (v_x + v_y - w) / denominator;

    m_chassis.setWheelVelocities(leftFront, leftBack, rightBack, rightFront);
}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    m_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
