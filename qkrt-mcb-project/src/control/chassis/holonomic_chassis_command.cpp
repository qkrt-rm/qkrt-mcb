#include "holonomic_chassis_command.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(Drivers &drivers, HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& m_operatorInterface)
    : m_chassis(chassis),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger)

{
    addSubsystemRequirement(&chassis);
}

void HolonomicChassisCommand::initialize()
{
}

void HolonomicChassisCommand::execute()
{       
        m_operatorInterface.pollInputDevices();

        volatile communication::NavData data = m_visionCoprocessor.getNavData();

        float xInp = data.xVel * 10.0f;
        float yInp = data.yVel * 10.0f;

        m_logger.printf("Message Recieved: x=%.3f y= %.3f\n", static_cast<double>(xInp), static_cast<double>(yInp));
        m_logger.delay(200);


        //float xInp = m_operatorInterface.getChassisXInput() * REMOTE_SENSITIVITY;
        //float yInp = m_operatorInterface.getChassisYInput() * REMOTE_SENSITIVITY;
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
