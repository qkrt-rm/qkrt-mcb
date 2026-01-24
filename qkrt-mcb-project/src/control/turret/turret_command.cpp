#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers & drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& m_operatorInterface)
    : m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      m_isAutoAim(true),
      m_pitchSensitivity(1.0f), m_yawSensitivity(1.0f),
      m_globalYawTarget(0.0f), m_globalPitchTarget(0.0f),
      m_lastTarget{NAN, NAN, NAN}
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    volatile communication::TurretData currentTarget = m_visionCoprocessor.getTurretData();

    bool isNewData = (currentTarget.xPos != m_lastTarget.xPos 
                        || currentTarget.yPos != m_lastTarget.yPos
                        || currentTarget.zPos != m_lastTarget.zPos);

    if (m_isAutoAim)
    {
        //AIM Command once target is found 
        
        if(isNewData)
        {
            m_turret.lock();        //tells subsytem to lock on target not used atm

            float aimYawRelative = std::atan2(currentTarget.yPos, currentTarget.xPos);
            float aimPitchRelative = std::atan2(std::hypot(currentTarget.yPos, currentTarget.xPos), currentTarget.zPos); 
           
            m_lastTarget.xPos = currentTarget.xPos;
            m_lastTarget.yPos = currentTarget.yPos;
            m_lastTarget.zPos = currentTarget.zPos;  

            m_globalYawTarget = m_turret.getYaw() + aimYawRelative * -1.0f;
            m_globalPitchTarget = 1.6f + aimPitchRelative * -1.0f;
        }

        m_turret.setYaw(m_globalYawTarget);
        m_turret.setPitch(m_globalPitchTarget);

        // m_logger.printf("CurrentYaw= %.3f | ExpectedYaw= %.3f \n", static_cast<double>(currentAzimuth), static_cast<double>(aimAzimuth));
        // m_logger.printf("Target Data: X= %.3f | Y= %.3f | Z= %.3f \n", static_cast<double>(xPos), static_cast<double>(yPos), static_cast<double>(zPos));
        
        // if(xPos == 0.0f){
        //      m_turret.setPitch(0.0f);
        // } else {
        //     m_turret.setPitch(1.6f + aimElevation * -1.0f);
        // }
        // if(currentAzimuth > 3.14159f){
        //     m_turret.setYaw((currentAzimuth - 6.28318f) + aimAzimuth * -1.0f);
        //     m_logger.printf("yaw= %.3f, currentYaw = %.3f \n", static_cast<double>((currentAzimuth - 6.28318f) + aimAzimuth * -1.0f), static_cast<double>(aimAzimuth));
        // } else {
        //     m_turret.setYaw(currentAzimuth + aimAzimuth * -1.0f);
        //     m_logger.printf("yaw= %.3f, currentYaw = %.3f \n", static_cast<double>(currentAzimuth + aimAzimuth * -1.0f), static_cast<double>(aimAzimuth));
        // }
        // m_logger.delay(400);
        
    }
    else
    {
        //Manual Velocity Control 
        m_operatorInterface.pollInputDevices();

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