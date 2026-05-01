#include "m3508_velocity_agitator_subsystem.hpp"
#include <algorithm>

namespace control::agitator::m3508
{

M3508AgitatorSubsystem::M3508AgitatorSubsystem(Drivers& drivers, const agitatorConfig& config)
    : tap::control::Subsystem(&drivers),
      m_desiredOutput(),
      m_pidController(),
      m_motor({
            Motor(&drivers, config.agitatorId, config.canBus, false, "agitator"), 
      }),
      m_drivers(&drivers)
{
    m_pidController.setParameter(config.agitatorVelocityPidConfig);   
}

void M3508AgitatorSubsystem::initialize()
{
    m_motor.initialize();
}

void M3508AgitatorSubsystem::setAgitatorSpeed(float speed)
{
    m_desiredOutput = speed;
}

void M3508AgitatorSubsystem::refresh()
{
    ///
    /// @brief uses a wheel's proportional-integral-derivative controller (PID controller)
    /// and desired output to calculate the output current needed to spin the wheel's motor
    /// at the desired speed.
    ///
    /// @param pid the wheel's proportional-integral-derivative controller
    /// @param motor the wheel's motor
    /// @param desiredOutput the wheel's desired output in Rps
    ///

    if (m_drivers->isEmergencyStopActive()) {
        m_pidController.reset();
        m_pidController.update(0.0f);
    }
    else {
        m_pidController.update(m_desiredOutput - m_motor.getEncoder()->getVelocity());
    }
    float desiredCurrent = std::clamp(m_pidController.getValue(), -M3508::MAX_CURRENT, M3508::MAX_CURRENT);
    m_motor.setDesiredOutput(desiredCurrent);

}  

}// namespace control::agitator::m3508