#include "m3508_agitator_command.hpp"

#include "tap/control/command.hpp"

#include "m3508_velocity_agitator_subsystem.hpp"

#include "control/control_operator_interface.hpp"

namespace control::agitator::m3508
{
    M3508AgitatorCommand::M3508AgitatorCommand(
        Drivers& drivers, M3508AgitatorSubsystem& agitator, 
        float speed, tap::control::Command* flywheelsCommand, ControlOperatorInterface& operatorInterface)
        :   m_drivers(&drivers),
            m_agitator(agitator),
            m_agitatorSpeed(speed),      
            m_flywheelsCommand(flywheelsCommand),
            m_operatorInterface(operatorInterface)
    {
        addSubsystemRequirement(&agitator);
    }

    bool M3508AgitatorCommand::isReady() 
    {
        //check flywheels are on before agitator command
        return m_drivers->commandScheduler.isCommandScheduled(m_flywheelsCommand);
    }

    bool M3508AgitatorCommand::isFinished() const
    {
        //trigger end command when flywheels are off
        return !m_drivers->commandScheduler.isCommandScheduled(m_flywheelsCommand);
    }

    void M3508AgitatorCommand::execute() 
    {

        float targetSetpoint = m_agitatorSpeed;

        if (m_operatorInterface.isHeroRevAgitator())
        {
            targetSetpoint = -m_agitatorSpeed * 0.2f; 
        }

        m_agitator.setAgitatorSpeed(targetSetpoint);
    } 

    void M3508AgitatorCommand::end(bool)
    {
        m_agitator.setAgitatorSpeed(OFF_SPEED);
    }
 

} //namespace control::agitator::m3508