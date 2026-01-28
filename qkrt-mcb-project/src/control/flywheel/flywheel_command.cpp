#include "flywheel_command.hpp"

#include "tap/control/command.hpp"

#include "flywheel.hpp"

#include "control/control_operator_interface.hpp"

namespace control::flywheel
{
    FlywheelOnCommand::FlywheelOnCommand(FlywheelSubsystem& flywheel, float flywheel_speed) :
    m_flywheel(flywheel), m_flywheelSpeed(flywheel_speed)
    {
        addSubsystemRequirement(&m_flywheel);
    }

    void FlywheelOnCommand::initialize() {}

    void FlywheelOnCommand::execute() 
    {
        m_flywheel.setDesiredOutput(m_flywheelSpeed);
    } 

    void FlywheelOnCommand::end(bool)
    {
        m_flywheel.setDesiredOutput(OFF_SPEED);
    }

    bool FlywheelOnCommand::isFinished() const {return false;}

} //namespace control::flywheel

