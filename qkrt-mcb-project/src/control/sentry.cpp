#include "sentry.hpp"

using tap::can::CanBus;
using tap::motor::MotorId;

namespace control
{

Robot::Robot(Drivers& drivers)
    : m_drivers(drivers),
      m_chassis(drivers,
                 chassis::ChassisConfig {
                     .leftFrontId  = MotorId::MOTOR1,
                     .leftBackId   = MotorId::MOTOR2,
                     .rightBackId  = MotorId::MOTOR3,
                     .rightFrontId = MotorId::MOTOR4,
                     .canBus       = CanBus::CAN_BUS1,
                     .wheelVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000), // TODO: tune this
                 }),
      m_chassisCommand(drivers, m_chassis, m_turret, drivers.controlOperatorInterface),
      m_turret(drivers,
                turret::TurretConfig {
                    .pitchId = MotorId::MOTOR6,
                    .yawId   = MotorId::MOTOR5,
                    .pitchInverted = false,
                    .yawInverted = true,
                    .canBus  = CanBus::CAN_BUS1,
                    .yawForwardOffset = 5455u,
                    .pitchHorizontalOffset = 0u,  // TODO: get this number when pitch motor is mounted
                }),
      m_turretCommand(drivers, m_turret, drivers.controlOperatorInterface),
      m_flywheels(drivers),
      m_flywheelsCommand(m_flywheels, 0.39f),
      m_agitator(drivers,
                agitator::agitatorConfig{
                    .agitatorId = MotorId::MOTOR7,
                    .canBus = CanBus::CAN_BUS1,
                    .agitatorVelocityPidConfig = modm::Pid<float>::Parameter(1000, 0, 0, 0, 16000), 
                }),
     m_agitatorCommand(m_agitator, -15.0)
{
}

void Robot::initialize()
{
    initializeSubsystems();
    registerSubsystems();
    setDefaultCommands();
    startCommands();
    registerIoMappings();

    m_drivers.visionCoprocessor.setChassisSubsystem(&m_chassis);
    m_drivers.visionCoprocessor.setTurretSubsystem(&m_turret);
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

    //don't continously run these commands
    //m_flywheels.setDefaultCommand(&m_flywheelsCommand);
    //m_velocityAgitatorSubsystem.setDefaultCommand(&m_agitatorCommand);
}

void Robot::startCommands()
{
}

void Robot::registerIoMappings()
{
    //Flywheel and Agitator Mapping
    m_drivers.commandMapper.addMap(& m_leftSwitchUP);

}



}  // namespace control