#include "holonomic_chassis_command.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::chassis
{

HolonomicChassisCommand::HolonomicChassisCommand(Drivers &drivers, HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& m_operatorInterface,
                                                 chassisCommandConfig config)
    : m_chassis(chassis),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_maxSpeed(config.maxChassisSpeed),
      m_chassisRotSpeed(config.maxRotSpeed),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      m_drivers(&drivers)

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
        bool isAutoNav = m_operatorInterface.getAutoNavInput();
        auto gameData = m_drivers->refSerial.getGameData();

        float rawInpX = (isAutoNav && gameData.gameStage == tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME) ? 
                            data.xVel * 1.5f : m_operatorInterface.getChassisXInput();
        float rawInpY = (isAutoNav && gameData.gameStage == tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME) ? 
                            data.yVel * 1.5f : m_operatorInterface.getChassisYInput();

        // DEBUG
        // float rawInpX = (isAutoNav) ? data.xVel * 1.5f : m_operatorInterface.getChassisXInput();
        // float rawInpY = (isAutoNav) ? data.yVel * 1.5f : m_operatorInterface.getChassisYInput();
        
        // m_logger.printf("Message Recieved: x=%.3f y= %.3f\n", static_cast<double>(xInp), static_cast<double>(yInp));
        // m_logger.delay(200);

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
        float w = (isAutoNav && gameData.gameStage == tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME) ? 
                        m_chassisRotSpeed : m_chassisRotSpeed * m_operatorInterface.getChassisBeyblade();
        
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
