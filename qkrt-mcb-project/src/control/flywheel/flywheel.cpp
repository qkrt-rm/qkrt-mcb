#include "flywheel.hpp"

namespace control::flywheel
{
    FlywheelSubsystem::FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config)
        : tap::control::Subsystem(&drivers), 
        m_desiredOutput(),
        m_motors({
                Motor(&drivers, config.rightFlywheelId, config.canBus, false, "RFly"),
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

    void FlywheelSubsystem::setDesiredOuput(float leftFlywheel, float rightFlywheel) 
    {
        m_motors[static_cast<u_int8_t>(MotorId::LFly)].setDesiredOutput(leftFlywheel);
        m_motors[static_cast<u_int8_t>(MotorId::RFly)].setDesiredOutput(rightFlywheel);
    }

    void FlywheelSubsystem::refresh() 
    {
        //do PID
    }

} // namespace control::flywheel
