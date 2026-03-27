#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers & drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& m_operatorInterface)
    : m_drivers(drivers),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      isAutoAim(true),
      m_globalPitch(0.0f), m_globalYaw(0.0f),
      m_pitchCommand(0.0f), m_yawCommand(0.0f),
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
                        || currentTarget.zPos != m_lastTarget.zPos) && currentTarget.xPos != 0;

    if (m_operatorInterface.isAutoAim())
    {
        m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, true);

        //AIM Command once target is found 
        
        if(isNewData)
        {
            m_turret.lock();        //tells subsytem to lock on target not used atm

            float targetYpos = currentTarget.yPos - 0.095f;  //magic offset

            float aimYawRelative = std::atan2(targetYpos * -1, currentTarget.xPos);
            float aimPitchRelative = std::atan2(currentTarget.zPos, std::hypot(targetYpos, currentTarget.xPos));

            m_lastTarget.xPos = currentTarget.xPos;
            m_lastTarget.yPos = targetYpos;
            m_lastTarget.zPos = currentTarget.zPos;  
            
            m_globalPitchTarget = aimPitchRelative; 
            m_globalYawTarget = m_turret.getYaw() + aimYawRelative;   

            m_globalXTarget = currentTarget.xPos;
            
        }
        
        m_turret.getXTarget(m_globalXTarget);
        m_turret.setYaw(m_globalYawTarget);
        m_turret.setPitch(m_globalPitchTarget);
    }
    else
    {
        m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, false);

        //Manual Velocity Control 
        m_operatorInterface.pollInputDevices();

        m_turret.unlock(); 

        float pitchInp = m_operatorInterface.getTurretPitchInput();
        float yawInp = m_operatorInterface.getTurretYawInput();
        
        m_pitchCommand = pitchInp * (2.0f * static_cast<float>(M_PI)) * m_pitchSensitivity * (turret::TurretSubsystem::DT);
        m_globalPitch += m_pitchCommand;

        m_turret.setPitch(m_globalPitch);
        m_turret.setYawRps(yawInp);

        m_turret.ChassisRot(m_operatorInterface.isChassisBeyblade());
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret