#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers & drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& m_operatorInterface)
    : m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      isAutoAim(true),
      m_pitchSensitivity(1.0f), m_yawSensitivity(1.0f),
      m_globalYawTarget(0.0f), m_globalPitchTarget(0.0f),
      m_lastFiveTargetsVelocity{{0.0f, 0.0f, 0.0f}},
      m_acceleration{0.0f, 0.0f, 0.0f}
      
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
        //AIM Command once target is found 
        
        if(isNewData)
        {
            m_turret.lock();        //tells subsytem to lock on target not used atm

            float targetYpos = currentTarget.yPos - 0.095f;  //magic offset
            
            float dt = 0.002f; //TODO: calculate real dt using timestamps

            m_lastFiveTargetsVelocity[0][0] = (currentTarget.xPos - m_lastPosition[0]) / dt;
            m_lastFiveTargetsVelocity[0][1] = (currentTarget.yPos - m_lastPosition[1]) / dt;
            m_lastFiveTargetsVelocity[0][2] = (currentTarget.zPos - m_lastPosition[2]) / dt;

            findTargetProjectileIntersection(
                tap::algorithms::ballistics::SecondOrderKinematicState(
                    modm::Vector3f(currentTarget.xPos, currentTarget.yPos, currentTarget.zPos),
                    modm::Vector3f(m_lastFiveTargetsVelocity[0][0], m_lastFiveTargetsVelocity[0][1], m_lastFiveTargetsVelocity[0][2]),
                    modm::Vector3f(0.0f, 0.0f, 0.0f)),
                20.0f, //bullet velocity in m/s TODO: get real value
                1, //num iterations
                &m_globalPitchTarget,
                &m_globalYawTarget,
                nullptr, //projected travel time not needed for control
                0.0f //pitch axis offset TODO: get real value
            );

            m_globalPitchTarget = m_globalPitchTarget * -1;
            m_globalYawTarget = m_globalYawTarget * -1;
            
            // float aimYawRelative = std::atan2(targetYpos * -1, currentTarget.xPos);
            // float aimPitchRelative = std::atan2(currentTarget.zPos, std::hypot(targetYpos, currentTarget.xPos));

            // m_lastTarget.xPos = currentTarget.xPos;
            // m_lastTarget.yPos = targetYpos;
            // m_lastTarget.zPos = currentTarget.zPos;
            
            // m_globalPitchTarget = aimPitchRelative; 
            // m_globalYawTarget = m_turret.getYaw() + aimYawRelative;   

            // m_lastPosition[0] = currentTarget.xPos;
            // m_lastPosition[1] = currentTarget.yPos;
            // m_lastPosition[2] = currentTarget.zPos;

        }
        
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