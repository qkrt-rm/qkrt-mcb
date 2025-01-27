#include "sentry.hpp"

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
                     .wheelVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000),
                 }),
      _M_chassisCommand(_M_chassis, drivers.controlOperatorInterface),
      _M_turret(drivers,
                turret::TurretConfig {
                    .pitchId = MotorId::MOTOR6,
                    .yawId   = MotorId::MOTOR5,
                    .canBus  = CanBus::CAN_BUS1,
                    .turretVelocityPidConfig = modm::Pid<float>::Parameter(500, 3, 0, 70, 100000),
                }),
      _M_turretCommand(_M_turret, drivers.controlOperatorInterface, drivers.uart)
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
}

void Robot::registerSubsystems()
{
    _M_drivers.commandScheduler.registerSubsystem(&_M_chassis);
    _M_drivers.commandScheduler.registerSubsystem(&_M_turret);
}

void Robot::setDefaultCommands()
{
    _M_chassis.setDefaultCommand(&_M_chassisCommand);
    _M_turret.setDefaultCommand(&_M_turretCommand);
}

void Robot::startCommands()
{
}

void Robot::registerIoMappings()
{
}

}  // namespace control