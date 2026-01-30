#include "flywheel_on_com_dd.hpp"

#include "tap/control/command.hpp"

#include "flywheel_sub_dd.hpp"

#include "control/control_operator_interface.hpp"

namespace control::flywheel
{
    FlywheelOnCommand::FlywheelOnCommand(FlywheelSubsystem& flywheel, float flywheel_speed) :
    m_flywheel(flywheel), m_flywheelSpeed(flywheel_speed)
    {
        addSubsystemRequirement(&flywheel);
    }

    void FlywheelOnCommand::initialize() {}

    void FlywheelOnCommand::execute() 
    {
        m_flywheel.setWheelVelocities(m_flywheelSpeed);
    } 

    void FlywheelOnCommand::end(bool)
    {
        m_flywheel.setWheelVelocities(OFF_SPEED);
    }


} //namespace control::flywheel