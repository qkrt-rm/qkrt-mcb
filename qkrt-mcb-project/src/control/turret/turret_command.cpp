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

<<<<<<< HEAD
        m_turret.setElevation(desiredElevation);
        m_turret.setAzimuth(desiredAzimuth);

=======
    // curr = clock::getTimeMicroseconds();
    // dt = curr - prev;
    // acc += static_cast<float>(dt * Speed) * 1e-6;
    // prev = curr;
    if (_M_operatorInterface.getEmergencyStopInput()) {
        
        float desiredElevation = 0.0f;
        float desiredAzimuth = 0.0f;

        _M_turret.setElevation(desiredElevation);
        _M_turret.setAzimuth(desiredAzimuth);

>>>>>>> c1c2d20 (Added Emergency stop function.)
        // Should this be empty?


    }
<<<<<<< HEAD
    else if (m_target != nullptr)
=======
    else if (_M_target == nullptr)
>>>>>>> c1c2d20 (Added Emergency stop function.)
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