#include "sentry.hpp"
#include "flywheel/flywheel_subsystem.hpp"
#include "flywheel/flywheel_on_command.hpp"


using tap::can::CanBus;
using tap::motor::MotorId;

namespace control
{

Robot::Robot(Drivers& drivers)
    : _M_drivers(drivers),
      _M_chassis(drivers,
                 chassis::ChassisConfig {
                     .leftFrontId  = MotorId::MOTOR1,
                     .leftBackId   = MotorId::MOTOR2,
                     .rightBackId  = MotorId::MOTOR3,
                     .rightFrontId = MotorId::MOTOR4,
                     .canBus       = CanBus::CAN_BUS1,
                     .wheelVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000), // TODO: tune this
                 }),
      _M_chassisCommand(_M_chassis, _M_turret, drivers.controlOperatorInterface),
      _M_turret(drivers,
                turret::TurretConfig {
                    .pitchId = MotorId::MOTOR6,
                    .yawId   = MotorId::MOTOR5,
                    .pitchInverted = false,
                    .yawInverted = true,
                    .canBus  = CanBus::CAN_BUS1,
                    .yawForwardOffset = 5455u,
                    .pitchHorizontalOffset = 0u,  // TODO: get this number when pitch motor is mounted
                }),
      _M_turretCommand(_M_turret, drivers.controlOperatorInterface, drivers.uart),
      _M_flywheels(drivers),
      _M_flywheelsCommand(&_M_flywheels, drivers.controlOperatorInterface, 0.39f),
      _M_agitator(&drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, true, "e"),
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
        _M_velocityAgitatorSubsystem(drivers, eduPidConfig, _M_agitator), // FIX LATER
        moveIntegralCommand(_M_velocityAgitatorSubsystem, moveIntegralConfig),
        _M_agitatorCommand(_M_velocityAgitatorSubsystem, drivers.controlOperatorInterface, 38)
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
    _M_chassis.initialize();
    _M_turret.initialize();
    _M_flywheels.initialize();
    _M_agitator.initialize();
}

void Robot::registerSubsystems()
{
    _M_drivers.commandScheduler.registerSubsystem(&_M_chassis);
    _M_drivers.commandScheduler.registerSubsystem(&_M_turret);
    _M_drivers.commandScheduler.registerSubsystem(&_M_flywheels);
    _M_drivers.commandScheduler.registerSubsystem(&_M_velocityAgitatorSubsystem);
}

void Robot::setDefaultCommands()
{
    _M_chassis.setDefaultCommand(&_M_chassisCommand);
    _M_turret.setDefaultCommand(&_M_turretCommand);
    _M_flywheels.setDefaultCommand(&_M_flywheelsCommand);
    _M_velocityAgitatorSubsystem.setDefaultCommand(&_M_agitatorCommand);
}

void Robot::startCommands()
{
}

void Robot::registerIoMappings()
{
}

}  // namespace control