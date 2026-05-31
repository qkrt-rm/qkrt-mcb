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
      //
      m_currentState(SentryState::SCANNING),
      m_targetLostTicks(0),
      m_scanDirection(1.0f)
      //

{
    addSubsystemRequirement(&turret);
}

void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    volatile communication::TurretData currentTarget = m_visionCoprocessor.getTurretData();

    bool hasValidTarget = (currentTarget.xPos != 0);
    // bool isNewData = (currentTarget.xPos != m_lastTarget.xPos 
    //                     || currentTarget.yPos != m_lastTarget.yPos
    //                     || currentTarget.zPos != m_lastTarget.zPos) && currentTarget.xPos != 0;
    
    //     if(isNewData) --> used instead of has valid target in phase 2
    
    // -----------------------------------------
    // Phase 1: State Transitions
    // -----------------------------------------
    if (m_operatorInterface.isAutoAim())
    {
        switch (m_currentState)
        {
            case SentryState::SCANNING:
                if (hasValidTarget)
                {
                    // Target acquired: immediately switch to shooting
                    m_currentState = SentryState::SHOOTING;
                    m_targetLostTicks = 0; 
                }
                break;

            case SentryState::SHOOTING:
                if (!hasValidTarget)
                {
                    // Target lost: start counting ticks
                    m_targetLostTicks++;
                    if (m_targetLostTicks >= TARGET_LOST_TIMEOUT_TICKS)
                    {
                        // Timeout reached: revert to scanning
                        m_currentState = SentryState::SCANNING;
                    }
                }
                else
                {
                    // Target is actively tracked: reset the grace period timer
                    m_targetLostTicks = 0; 
                }
                break;
        }
    }
    else
    {
        // Fallback if auto-aim is disabled
        m_currentState = SentryState::SCANNING; 
    }

    // -----------------------------------------
    // Phase 2: State Behaviors
    // -----------------------------------------
    if (m_operatorInterface.isAutoAim())
    {
        if (m_currentState == SentryState::SHOOTING)
        {
            m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, true);
            m_turret.lock();

            // Only update the mathematical aim targets if we actually have fresh data
            // (If we are in the "timeout" grace period, we just hold the last known m_globalYawTarget)
            if (hasValidTarget)
            {
                m_turret.zeroYaw();

                float targetYpos = currentTarget.yPos - 0.095f;  // magic offset
                float aimYawRelative = std::atan2(targetYpos * -1, currentTarget.xPos);
                float aimPitchRelative = std::atan2(currentTarget.zPos, std::hypot(targetYpos, currentTarget.xPos));

                m_lastTarget.xPos = currentTarget.xPos;
                m_lastTarget.yPos = targetYpos;
                m_lastTarget.zPos = currentTarget.zPos;  
                
                m_globalPitchTarget = m_turret.getPitch() + aimPitchRelative; 
                m_globalYawTarget = aimYawRelative;   
            }
            
            m_turret.setYaw(m_globalYawTarget);
            m_turret.setPitch(m_globalPitchTarget);
        }

        else if (m_currentState == SentryState::SCANNING)
        {
            m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, false);
            m_turret.unlock();

            // Automatic sweeping movement logic (180 degrees total range)
            float currentYaw = m_turret.getYaw();
            
            // Limit is pi/2 radians (90 degrees) in either direction from 0
            const float SCAN_LIMIT = static_cast<float>(M_PI) / 2.0f; 
            const float SCAN_SPEED_RPS = 0.2f; // Adjust this for faster/slower sweeping
            
            // Reverse direction if we hit or exceed the limits
            if (currentYaw >= SCAN_LIMIT)
            {
                m_scanDirection = -1.0f;
            }
            else if (currentYaw <= -SCAN_LIMIT)
            {
                m_scanDirection = 1.0f;
            }

            // Command the turret to move at the set velocity
            m_turret.setYawRps(SCAN_SPEED_RPS * m_scanDirection);
        }
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

        m_globalPitch = std::clamp(m_globalPitch, m_turret.m_pitchDownLim, m_turret.m_pitchUpLim);

        m_turret.setPitch(m_globalPitch);
        m_turret.setYawRps(yawInp);

        m_turret.ChassisRot(m_operatorInterface.isChassisBeyblade());
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret