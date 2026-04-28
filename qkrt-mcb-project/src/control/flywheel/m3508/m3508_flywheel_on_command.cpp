#include "m3508_flywheel_on_command.hpp"

#include "tap/control/command.hpp"

#include "m3508_flywheel_subsystem.hpp"

#include "control/control_operator_interface.hpp"

namespace control::flywheel::m3508
{
    M3508FlywheelOnCommand::M3508FlywheelOnCommand(M3508FlywheelSubsystem& flywheel,
                                         float flywheel_speed)
    : m_flywheel(flywheel),
      m_flywheelSpeed(flywheel_speed)

    {
        addSubsystemRequirement(&flywheel);
    }

    void M3508FlywheelOnCommand::initialize() {}

    void M3508FlywheelOnCommand::execute() 
    {
        m_flywheel.setFlywheelVel(m_flywheelSpeed);
    } 

    void M3508FlywheelOnCommand::end(bool)
    {
        m_flywheel.setFlywheelVel(OFF_SPEED);
    }
 

} //namespace control::flywheel