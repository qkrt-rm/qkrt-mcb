#if defined(TARGET_STANDARDISO)

#include "standard_1v1.hpp"

namespace control
{
Robot::Robot(Drivers& drivers)
    : m_drivers(drivers),
      m_chassis(drivers, m_chassisConfig),
      m_chassisCommand(drivers, m_chassis, m_turret, drivers.controlOperatorInterface, m_chassisCommandConfig),
      m_turret(drivers, m_turretConfig, m_chassisCommandConfig),
      m_turretCommand(drivers, m_turret, drivers.controlOperatorInterface),
      m_flywheels(drivers, m_flywheelConfig),
      m_flywheelsCommand(m_flywheels, m_flyhweelSpeed),
      m_agitator(drivers, m_agitatorConfig),
     m_agitatorCommand(drivers, m_agitator, m_agitatorSpeed, false, &m_flywheelsCommand, drivers.controlOperatorInterface)    
{
}

void Robot::initialize()
{
    initializeSubsystems();
    registerSubsystems();
    setDefaultCommands();
    startCommands();
    registerIoMappings();
}


void Robot::initializeSubsystems()
{
    m_chassis.initialize();
    m_turret.initialize();
    m_flywheels.initialize();
    m_agitator.initialize();
}

void Robot::registerSubsystems()
{
    m_drivers.commandScheduler.registerSubsystem(&m_chassis);
    m_drivers.commandScheduler.registerSubsystem(&m_turret);
    m_drivers.commandScheduler.registerSubsystem(&m_flywheels);
    m_drivers.commandScheduler.registerSubsystem(&m_agitator);
}

void Robot::setDefaultCommands()
{
    m_chassis.setDefaultCommand(&m_chassisCommand);
    m_turret.setDefaultCommand(&m_turretCommand);
}

void Robot::startCommands()
{
}

void Robot::registerIoMappings()
{
    //Flywheel and Agitator Mapping

    //remote
    m_drivers.commandMapper.addMap(& m_leftSwitchUP);
    m_drivers.commandMapper.addMap(& m_rightSwitchUP);

    //mouse
    m_drivers.commandMapper.addMap(& m_ToggleFlyX);
    m_drivers.commandMapper.addMap(& m_leftMouseIndex);

}
}  // namespace control

#endif  // TARGET_STANDARD_1v1