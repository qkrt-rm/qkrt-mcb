#include "sentry.hpp"
#include "flywheel/flywheel_subsystem.hpp"
#include "flywheel/flywheel_on_command.hpp"


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
      m_chassisCommand(m_chassis, m_turret, drivers.controlOperatorInterface),
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
      m_flywheelsCommand(m_flywheels, drivers.controlOperatorInterface, 0.39f),
      m_agitator(&drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, true, "e"),
        eduPidConfig{
            .kp = 1000,
            .ki = 0,
            .kd = 0,
            .maxICumulative = 0,
            .maxOutput = 16000
        },
        moveIntegralConfig{
            .targetIntegralChange = M_TWOPI / 10.0f, 
            .desiredSetpoint = M_TWOPI,
            .integralSetpointTolerance = 0
        },
        m_velocityAgitatorSubsystem(drivers, eduPidConfig, m_agitator), 
        moveIntegralCommand(m_velocityAgitatorSubsystem, moveIntegralConfig),
        m_agitatorCommand(m_velocityAgitatorSubsystem, drivers.controlOperatorInterface, 38)

        // rightSwitchUp(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP), false),
        // HCM(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)),
         




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
    m_drivers.commandScheduler.registerSubsystem(&m_velocityAgitatorSubsystem);
}

void Robot::setDefaultCommands()
{
    m_chassis.setDefaultCommand(&m_chassisCommand);
    m_turret.setDefaultCommand(&m_turretCommand);
    m_flywheels.setDefaultCommand(&m_flywheelsCommand);
    m_velocityAgitatorSubsystem.setDefaultCommand(&m_agitatorCommand);
}

void Robot::startCommands()
{
}

void Robot::registerIoMappings()
{
}

}  // namespace control