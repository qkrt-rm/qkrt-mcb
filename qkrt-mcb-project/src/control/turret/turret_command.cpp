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

        float desiredElevation = 0.0f;
        float desiredAzimuth = 0.0f;

        m_logger.printf("PITCH= %.3f | YAW= %.3f \n", static_cast<double>(aimElevation), static_cast<double>(aimAzimuth));
        m_logger.delay(400);
    
        m_turret.setElevation(1.6f + aimElevation * -1.0f); 
     
        // m_turret.setElevation(0.0f);
        m_turret.setAzimuth(aimAzimuth * -1);
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