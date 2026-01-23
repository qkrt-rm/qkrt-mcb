#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers & drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& m_operatorInterface)
    : m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      m_pitchSensitivity(1.0f), m_yawSensitivity(1.0f),
      m_targetYaw(0.0f),
      m_target(nullptr)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    volatile communication::TurretData data = m_visionCoprocessor.getTurretData();

    const float xConst = 0.0f;
    const float yConst = 0.0f;
    const float zConst = 0.0f;

    float xPos = data.xPos + xConst;
    float yPos = data.yPos + yConst;
    float zPos = data.zPos + zConst;

    //TODO: Why negative Y so small from husky

    float aimAzimuth = std::atan2(yPos, xPos);
    float groundDist = std::hypot(yPos, xPos);
    float aimElevation = std::atan2(groundDist, zPos); 

    m_operatorInterface.pollInputDevices();
    if (true)
    {
        //AIM Command once target is found 

        m_turret.lock();

        float currentAzimuth = m_turret.getAzimuth();
        float currentElevation = m_turret.getElevation();

        // m_logger.printf("CurrentYaw= %.3f | ExpectedYaw= %.3f \n", static_cast<double>(currentAzimuth), static_cast<double>(aimAzimuth));
        // m_logger.printf("Target Data: X= %.3f | Y= %.3f | Z= %.3f \n", static_cast<double>(xPos), static_cast<double>(yPos), static_cast<double>(zPos));
        
        if(xPos == 0.0f){
             m_turret.setElevation(0.0f);
        } else {
            m_turret.setElevation(1.6f + aimElevation * -1.0f);
        }
        if(currentAzimuth > 3.14159f){
            m_turret.setAzimuth((currentAzimuth - 6.28318f) + aimAzimuth * -1.0f);
            m_logger.printf("yaw= %.3f, currentYaw = %.3f \n", static_cast<double>((currentAzimuth - 6.28318f) + aimAzimuth * -1.0f), static_cast<double>(aimAzimuth));
        } else {
            m_turret.setAzimuth(currentAzimuth + aimAzimuth * -1.0f);
            m_logger.printf("yaw= %.3f, currentYaw = %.3f \n", static_cast<double>(currentAzimuth + aimAzimuth * -1.0f), static_cast<double>(aimAzimuth));
        }
        m_logger.delay(400);
        
    }
    else
    {
        //Manual Velocity Control 

        m_turret.unlock();

        float pitchInp = m_operatorInterface.getTurretPitchInput();
        float yawInp = m_operatorInterface.getTurretYawInput();
        
        //update setpoint to operator input
        m_turret.setPitchRps(pitchInp);
        m_turret.setYawRps(yawInp);
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret