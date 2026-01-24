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

            float aimYawRelative = std::atan2(currentTarget.yPos * -1, currentTarget.xPos);
            float aimPitchRelative = std::atan2(currentTarget.zPos, std::hypot(currentTarget.yPos, currentTarget.xPos));

            m_lastTarget.xPos = currentTarget.xPos;
            m_lastTarget.yPos = currentTarget.yPos;
            m_lastTarget.zPos = currentTarget.zPos;  
            
            m_globalPitchTarget = aimPitchRelative; 
            m_globalYawTarget = m_turret.getYaw() + 0.1 + aimYawRelative;   //magic offset
        }
        
        // m_logger.printf("CurrentYaw= %.3f | TargetYaw= %.3f \n", static_cast<double>(m_turret.getYaw()), static_cast<double>(m_globalYawTarget));
        // m_logger.delay(400);

        m_turret.setYaw(m_globalYawTarget);
        m_turret.setPitch(m_globalPitchTarget);
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