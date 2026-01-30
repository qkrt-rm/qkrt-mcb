#include "flywheel_sub_dd.hpp"

namespace control::flywheel
{
    FlywheelSubsystem::FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config)
        : tap::control::Subsystem(&drivers), 
        m_desiredOutput(),
        m_motors({
                Motor(&drivers, config.rightFlywheelId, config.canBus, false, "RFly"), //the false could be spinning it the other way.
                Motor(&drivers, config.leftFlyWheelId, config.canBus, true, "LFly"),
            }),
        m_drivers(&drivers)
        {
        for (auto& controller : m_pidControllers)
        {   
            controller.setParameter(config.wheelVelocityPidConfig);
        }

        }

    void FlywheelSubsystem::initialize() 
    {
        for (auto& motor : m_motors)
        {
            motor.initialize();
        }
    }

    void FlywheelSubsystem::setWheelVelocities(float flywheelSpeed) 
    {
        flywheelSpeed = std::clamp(flywheelSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        m_desiredOutput[static_cast<uint8_t>(MotorId::LFly)] = flywheelSpeed;
        m_desiredOutput[static_cast<uint8_t>(MotorId::RFly)] = flywheelSpeed;
    }

    void FlywheelSubsystem::refresh() 
    {
        auto runPid = [](Pid& pid, Motor& motor, float desiredOutput, Drivers *m_drivers_lf) -> void
        { 
            if (m_drivers_lf->isEmergencyStopActive()) {
                pid.reset();
                pid.update(0.0f);
            }
            else {
                pid.update(desiredOutput - motor.getShaftRPM());
            }
            motor.setDesiredOutput(pid.getValue());
        };

        /**
        * TODO: Power Limiting Logic
        */

        for (size_t ii = 0; ii < m_motors.size(); ii++)
        {
            runPid(m_pidControllers[ii], m_motors[ii], m_desiredOutput[ii],m_drivers);
        }
    }

} // namespace control::flywheel