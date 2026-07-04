#include "auto_holonomic_chassis_command.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::chassis
{

AutoHolonomicChassisCommand::AutoHolonomicChassisCommand(Drivers &drivers, HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& m_operatorInterface,
                                                 chassisCommandConfig config)
    : m_chassis(chassis),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_maxSpeed(config.maxChassisSpeed),
      m_chassisRotSpeed(config.maxRotSpeed),
      m_startTimer(0.0f),
      isNavReady(false),
      m_sequenceTimer(0.0f),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      m_drivers(&drivers),
      isHardCode(true),
      islockTurret(true),
      m_scriptStage(0)

{
    addSubsystemRequirement(&chassis);
}

void AutoHolonomicChassisCommand::initialize()
{
}

void AutoHolonomicChassisCommand::execute()
{       
        m_operatorInterface.pollInputDevices();

        volatile communication::NavData data = m_visionCoprocessor.getNavData();
        bool isAutoNav = m_operatorInterface.getAutoNavInput();
        auto gameData = m_drivers->refSerial.getGameData();
        auto robotData = m_drivers->refSerial.getRobotData();
        tap::communication::serial::RefSerialData::RobotId robotID = robotData.robotId;
        //blue right
        //red left
        isNavReady = false;
        islockTurret = true;
        //wait 3 seconds for nav to settle
        //gameData.gameStage == tap::communication::serial::RefSerialData::Rx::GameStage::IN_GAME &&
        if (!isNavReady)
        {
            m_startTimer += 0.002f;
            if (m_startTimer >= 3.0f)
            {
                isNavReady = true; 
            }
        }
        
        float rawInpX = (isAutoNav && isNavReady) ? 
                            data.xVel * 2.0f : m_operatorInterface.getChassisXInput();
        float rawInpY = (isAutoNav && isNavReady) ? 
                            data.yVel * 2.0f : m_operatorInterface.getChassisYInput();
        float w = 0;

        // if (isNavReady && isHardCode)
        // {
        //     m_sequenceTimer += 0.002f;

            //track cycles of 10s
            //float currentCycleTime = std::fmod(m_sequenceTimer, 10.0f);

            // move left

        if (isNavReady && isHardCode)
        {
             m_sequenceTimer += 0.002f;

            //track cycles of 10s
            //float currentCycleTime = std::fmod(m_sequenceTimer, 10.0f);

            // move left
            if (m_sequenceTimer < 5.5f)
            {
                rawInpX = 0.0f;

                if (robotID == tap::communication::serial::RefSerialData::RobotId::RED_SENTINEL)
                {
                    rawInpY = -0.5f;     
                }
                else
                {
                    rawInpY = 0.5f;      
                }
                w = 0;
            }
            // Station in Corner
            else
            {
                rawInpX = 0.0f;
                rawInpY = 0.0f;
                w = m_chassisRotSpeed;
                islockTurret = false;
            }
        }

            // if (m_sequenceTimer < 5.5f)
            // {
            //     rawInpX = 0.0f;

            //     if (robotID == tap::communication::serial::RefSerialData::RobotId::RED_SENTINEL)
            //     {
            //         rawInpY = 0.5f;     
            //     }
            //     else
            //     {
            //         rawInpY = -0.5f;      
            //     }
            //     w = 0;
            // }
            // } else if (m_sequenceTimer < 8.0 && m_scriptStage == 1){
            //     rawInpY = 0.0f;
            //     rawInpX = 0.5;
            //     w = 0;
            // } else if (m_sequenceTimer < 5.0 && m_scriptStage == 2){
            //     rawInpX = 0.0f;

            //     if (robotID == tap::communication::serial::RefSerialData::RobotId::RED_SENTINEL)
            //     {
            //         rawInpY = -0.5f;     
            //     }
            //     else
            //     {
            //         rawInpY = 0.5f;      
            //     }
            //     w = 0;
            // } else if (m_sequenceTimer < 2.5 && m_scriptStage == 3){
            //     rawInpY = 0.0f;
            //     rawInpX = 0.5;
            //     w = 0;
            // } else if (m_scriptStage == 4){
            //     rawInpX = 0.0f;
            //     rawInpY = 0.0f;
            //     w = 0;
            //     islockTurret = false;
            // }
            // // Station in Corner
        //     else
        //     {
        //         rawInpX = 0.0f;
        //         rawInpY = 0.0f;
        //         islockTurret = false;
        //         isHardCode = false;
        //         w = m_chassisRotSpeed;
        //     }
        // }
       

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
        //float w = (isAutoNav && isNavReady) ? 
        //                m_chassisRotSpeed : m_chassisRotSpeed * m_operatorInterface.getChassisBeyblade();
        
        float leftFront  = (v_x + v_y + w);
        float leftBack   = (v_x - v_y + w);
        float rightFront = (v_x - v_y - w);
        float rightBack  = (v_x + v_y - w);

        m_chassis.setWheelVelocities(leftFront, leftBack, rightBack, rightFront);
}

void AutoHolonomicChassisCommand::end(bool /* interrupted */)
{
    m_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace control::chassis
