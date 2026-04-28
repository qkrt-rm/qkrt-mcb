#include "m3508_flywheel_subsystem.hpp"
#include <algorithm>

namespace control::flywheel::m3508
{

M3508FlywheelSubsystem::M3508FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config)
    : tap::control::Subsystem(&drivers),
      m_desiredOutput(),
      m_pidControllers(),
      m_motors({
            Motor(&drivers, config.rightFlyId, config.canBus, true, "RFly"), 
            Motor(&drivers, config.leftFlyId, config.canBus, false, "LFly"),
      }),
      m_drivers(&drivers)
{
    for (auto& controller : m_pidControllers)
    {
        controller.setParameter(config.flyVelocityPidConfig);
    }
}

void M3508FlywheelSubsystem::initialize()
{
    for (auto& motor : m_motors)
    {
        motor.initialize();
    }
}

void M3508FlywheelSubsystem::setFlywheelVel(float speed)
{
        // mapping one range to another
        float a[2] = {0.0f, 1.0f}; // range1 
        float b[2] = {0.0f, M3508::MAX_CURRENT}; //range2
        speed = b[0] + (((speed - a[0])*(b[1]-b[0])) / (a[1]-a[0]));

        speed = std::clamp(speed, -M3508::MAX_CURRENT, M3508::MAX_CURRENT);
        m_desiredOutput[static_cast<uint8_t>(MotorId::LFly)] = speed;
        m_desiredOutput[static_cast<uint8_t>(MotorId::RFly)] = speed;
}

void M3508FlywheelSubsystem::refresh()
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
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput, Drivers *m_drivers_lf) -> void
    { 
        if (m_drivers_lf->isEmergencyStopActive()) {
            pid.reset();
            pid.update(0.0f);
        }
        else {
            pid.update(desiredOutput - motor.getEncoder()->getVelocity());
        }
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < m_motors.size(); ii++)
    {
        runPid(m_pidControllers[ii], m_motors[ii], m_desiredOutput[ii],m_drivers);
    }
}

}  // namespace control::flywheel::m3508