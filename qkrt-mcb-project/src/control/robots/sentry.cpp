#if defined(TARGET_SENTRY)

#include "sentry.hpp"

namespace control
{

Robot::Robot(Drivers& drivers)
    : m_drivers(drivers),
      m_chassis(drivers, m_chassisConfig),
      m_chassisCommand(drivers, m_chassis, m_turret, drivers.controlOperatorInterface, m_chassisCmdConfig),
      m_turret(drivers, turretConfig, m_chassisCmdConfig),
      m_turretCommand(drivers, m_turret, drivers.controlOperatorInterface, &m_flywheelsCommand, &m_agitatorCommand),
      m_flywheels(drivers),
      m_flywheelsCommand(m_flywheels, m_flywheelSpeed),
      m_agitator(drivers, m_agitatorConfig),
      m_agitatorCommand(drivers, m_agitator, m_agitatorSpeed, false, &m_flywheelsCommand, drivers.controlOperatorInterface)      //TUNE RATE
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
    m_drivers.visionCoprocessor.setChassisSubsystem(&m_chassis);
    m_drivers.visionCoprocessor.setTurretSubsystem(&m_turret);
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
    m_drivers.commandMapper.addMap(& m_rightMouseFlywheel);
    m_drivers.commandMapper.addMap(& m_leftMouseIndex);

}



}  // namespace control

#endif