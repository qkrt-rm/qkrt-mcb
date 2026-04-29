#if defined(TARGET_STANDARD)

#include "standard.hpp"

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
                    .pitchInverted = true,
                    .yawInverted = true,
                    .mcbHoriz = false,
                    .canBus  = CanBus::CAN_BUS1,
                    .yawForwardOffset = -4.7070f,          
                    .pitchHorizontalOffset = - 5.2102f,      
                    .pitchUpLim = 0.8360f,                  
                    .pitchDownLim = -0.3275f,               
                    .MAX_PITCH_POWER = GM6020::MAX_VOLTAGE,
                    .MAX_YAW_POWER = GM6020::MAX_VOLTAGE,
                    .MAX_RPS = GM6020::MAX_RPS,
                    .pitchPosGains = { .kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 500.0f, .maxOutput = GM6020::MAX_VOLTAGE },
                    .pitchVelGains = { .kp = 4000.0f, .ki = 110.0f, .kd = 0.0f, .maxICumulative = 3000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
                    .yawPosGains   = { .kp = 5.0f,  .ki = 0.0f, .kd = 0.0f, .maxICumulative = 5000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
                    .yawVelGains   = { .kp = 8000.0f, .ki = 10.0f,  .kd = 0.0f, .maxICumulative = 1000.0f, .maxOutput = GM6020::MAX_VOLTAGE }
                }),
      m_turretCommand(drivers, m_turret, drivers.controlOperatorInterface),
      m_flywheels(drivers, 
            flywheel::m3508::FlywheelConfig {
            .leftFlyId = MotorId::MOTOR1, 
            .rightFlyId = MotorId::MOTOR2, 
            .canBus = CanBus::CAN_BUS2,
            .flyVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000)
        }),
      m_flywheelsCommand(m_flywheels, 0.39f),
      m_agitator(drivers,
                agitator::agitatorConfig {
                    .agitatorId = MotorId::MOTOR3,
                    .canBus = CanBus::CAN_BUS2,
                    .agitatorVelocityPidConfig = modm::Pid<float>::Parameter(1000, 0, 0, 0, 16000), 
                }),
     m_agitatorCommand(m_agitator, -5.0)
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
    m_drivers.commandMapper.addMap(& m_leftSwitchUP);
    m_drivers.commandMapper.addMap(& m_rightSwitchUP);

}
}  // namespace control

#endif  // TARGET_STANDARD