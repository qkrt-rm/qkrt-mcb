#include "flywheel_sub_dd.hpp"

namespace control::flywheel
{
    FlywheelSubsystem::FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config)
        : tap::control::Subsystem(&drivers), 
        m_desiredOutput(),
        m_motors({
                Motor(&drivers, config.rightFlywheelId, config.canBus, false, "RFly"), //the false could be spinning it the other way.
                Motor(&drivers, config.leftFlyWheelId, config.canBus, false, "LFly"),
            }),
        m_drivers(&drivers)
        {

        }

    void FlywheelSubsystem::initialize() 
    {
        for (auto& motor : m_motors){
            motor.initialize();
        }
    }

    void FlywheelSubsystem::setWheelVelocities(float flywheelSpeed) 
    {
        m_desiredOutput[static_cast<uint8_t>(MotorId::LFly)] = flywheelSpeed;
        m_desiredOutput[static_cast<uint8_t>(MotorId::RFly)] = flywheelSpeed;
    }

    void FlywheelSubsystem::refresh() 
    {
        // // 1. Safety Check
        // if (m_drivers->isEmergencyStopActive()) {
        //     for (auto& motor : m_motors) motor.setDesiredOutput(0);
        //     return;
        // }
        m_motors[static_cast<uint8_t>(MotorId::LFly)].setDesiredOutput(m_desiredOutput[0]);
        m_motors[static_cast<uint8_t>(MotorId::RFly)].setDesiredOutput(m_desiredOutput[1]);
        // 2. Send the command to the hardware
        // for (size_t i = 0; i < m_motors.size(); ++i) {
        //     m_motors[i].setDesiredOutput(m_desiredOutput[i]);
        // }
    }

} // namespace control::flywheel