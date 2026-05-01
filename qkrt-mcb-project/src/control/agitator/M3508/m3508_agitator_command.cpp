#include "m3508_agitator_command.hpp"

#include "tap/control/command.hpp"

#include "m3508_velocity_agitator_subsystem.hpp"

#include "control/control_operator_interface.hpp"

namespace control::agitator::m3508
{
    M3508AgitatorCommand::M3508AgitatorCommand(M3508AgitatorSubsystem& agitator, float speed)
    : m_agitator(agitator),
      m_agitatorSpeed(speed)
    {
        addSubsystemRequirement(&agitator);
    }

    void M3508AgitatorCommand::initialize() {}

    void M3508AgitatorCommand::execute() 
    {
        m_agitator.setAgitatorSpeed(m_agitatorSpeed);
    } 

    void M3508AgitatorCommand::end(bool)
    {
        m_agitator.setAgitatorSpeed(OFF_SPEED);
    }
 

} //namespace control::agitator::m3508