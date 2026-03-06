#include "flywheel_on_command.hpp"

#include "tap/control/command.hpp"

#include "flywheel_subsystem.hpp"

#include "control/control_operator_interface.hpp"

namespace control::flywheel
{
    FlywheelOnCommand::FlywheelOnCommand(FlywheelSubsystem* flywheel,
                                         float flywheel_speed)
    : m_flywheel(flywheel),
      m_flywheelSpeed(flywheel_speed)

    {
        addSubsystemRequirement(flywheel);
    }

    void FlywheelOnCommand::initialize() {}

    void FlywheelOnCommand::execute() 
    {
        m_flywheel->setTargetSpeed(m_flywheelSpeed);
    } 

    void FlywheelOnCommand::end(bool)
    {
        m_flywheel->setTargetSpeed(OFF_SPEED);
    }
 

} //namespace control::flywheel