#include "holonomic_chassis_command.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& m_operatorInterface,
                                                 chassisCommandConfig config)
    : m_chassis(chassis),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_maxSpeed(config.maxChassisSpeed),
      m_chassisRotSpeed(config.maxRotSpeed)
{
    addSubsystemRequirement(&chassis);
}

void HolonomicChassisCommand::initialize()
{
}

void HolonomicChassisCommand::execute()
{       
        m_operatorInterface.pollInputDevices();

        float rawInpX = m_operatorInterface.getChassisXInput();
        float rawInpY = m_operatorInterface.getChassisYInput();

        Vector2f moveVector(rawInpX, rawInpY);
        float inputLength = moveVector.getLength();

        if (inputLength > 1.0f)
        {
            moveVector = moveVector / inputLength;
        }

        Vector2f scaledMove = moveVector * m_maxSpeed;

        float xInp = scaledMove.x;
        float yInp = scaledMove.y;
        float yawAngle = m_turret.getYaw();
        
        //compute rotation transformation
        float v_y = yInp * std::cos(-yawAngle) - xInp * std::sin(-yawAngle);
        float v_x = yInp * std::sin(-yawAngle) + xInp * std::cos(-yawAngle);
        float w = m_chassisRotSpeed * m_operatorInterface.getChassisBeyblade();
        
        float leftFront  = (v_x + v_y + w);
        float leftBack   = (v_x - v_y + w);
        float rightFront = (v_x - v_y - w);
        float rightBack  = (v_x + v_y - w);

        m_chassis.setWheelVelocities(leftFront, leftBack, rightBack, rightFront);
}

void HolonomicChassisCommand::end(bool /* interrupted */)
{
    m_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
