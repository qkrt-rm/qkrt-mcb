#include "m3508_flywheel.hpp"
#include <algorithm> // Required for std::clamp

namespace control::flywheel
{
    
    M3508FlywheelSubsystem::M3508FlywheelSubsystem(Drivers& drivers, const M3508FlywheelConfig& config)
        : FlywheelSubsystem(drivers), 
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

    void M3508FlywheelSubsystem::initialize() 
    {
        for (auto& motor : m_motors)
        {
            motor.initialize();
        }
    }

    void M3508FlywheelSubsystem::setTargetSpeed(float speed) 
    {
        // mapping one range to another
        float a[2] = {0.0f, 1.0f}; // range1 
        float b[2] = {0.0f, MAX_WHEELSPEED_RPM}; //range2
        speed = b[0] + (((speed - a[0])*(b[1]-b[0])) / (a[1]-a[0]));

        speed = std::clamp(speed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        m_desiredOutput[static_cast<uint8_t>(MotorId::LFly)] = speed;
        m_desiredOutput[static_cast<uint8_t>(MotorId::RFly)] = speed;
    }

    void M3508FlywheelSubsystem::refresh() 
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