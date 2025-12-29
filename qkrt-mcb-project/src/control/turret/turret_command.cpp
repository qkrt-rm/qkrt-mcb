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
    
    m_operatorInterface.pollInputDevices();
    if (m_operatorInterface.getEmergencyStopInput()) {
        
        float desiredElevation = 0.0f;
        float desiredAzimuth = 0.0f;

        m_turret.setElevation(desiredElevation);
        m_turret.setAzimuth(desiredAzimuth);

        // Should this be empty?


    }
    else if (m_target != nullptr)
    {
        //AIM Command once target is found 

        m_turret.lock();

        float desiredElevation = 0.0f;
        float desiredAzimuth = 0.0f;

        m_turret.setElevation(desiredElevation);
        m_turret.setAzimuth(desiredAzimuth);
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