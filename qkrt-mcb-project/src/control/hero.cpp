#if defined(TARGET_HERO)

#include "hero.hpp"

using tap::can::CanBus;
using tap::motor::MotorId;

namespace control
{

Robot::Robot(Drivers& drivers)
    :   m_drivers(drivers),
        m_chassis(drivers,
                 chassis::ChassisConfig {
                     .leftFrontId  = MotorId::MOTOR1,
                     .leftBackId   = MotorId::MOTOR2,
                     .rightBackId  = MotorId::MOTOR3,
                     .rightFrontId = MotorId::MOTOR4,
                     .canBus       = CanBus::CAN_BUS1,
                     .wheelVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000), // TODO: tune this
                 }),
        m_chassisCommand(drivers, m_chassis, m_turret, drivers.controlOperatorInterface,
                 chassis::chassisCommandConfig {
                     .maxChassisSpeed = 0.78f,
                     .maxRotSpeed = 0.65f
                }),
        m_turret(drivers,
                turret::TurretConfig {
                    .pitchId = MotorId::MOTOR6,
                    .yawId   = MotorId::MOTOR5,
                    .pitchGearRatio = GM6020::GEAR_RATIO,
                    .yawGearRatio = M3508::GEAR_RATIO * BELT_GEAR_RATIO,
                    .pitchInverted = true,
                    .yawInverted = true,
                    .isYawZeroed = false,       // need to point turret north on startup 
                    .mcbHoriz = true,
                    .canBus  = CanBus::CAN_BUS1,
                    .yawForwardOffset = 0.0f,         
                    .pitchHorizontalOffset = -2.10232f,      
                    .pitchUpLim = 0.16951f,                  
                    .pitchDownLim = -0.49241f,               
                    .MAX_PITCH_POWER = M3508::MAX_CURRENT,
                    .MAX_YAW_POWER = M3508::MAX_CURRENT,
                    .MAX_RPS = GM6020::MAX_RPS,
                    .pitchPosGains = { .kp = 15.5f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 500.0f, .maxOutput = GM6020::MAX_VOLTAGE },
                    .pitchVelGains = { .kp = 6000.0f, .ki = 110.0f, .kd = 0.0f, .maxICumulative = 3000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
                    .yawPosGains   = { .kp = 5.0f,  .ki = 0.0f, .kd = 0.0f, .maxICumulative = 5000.0f, .maxOutput = M3508::MAX_CURRENT },
                    .yawVelGains   = { .kp = 2500.0f, .ki = 100.0f,  .kd = 0.0f, .maxICumulative = 1000.0f, .maxOutput = M3508::MAX_CURRENT },
                    .yawFF = 1435.0f,
                    .pitchFF = 1000.0f,  
                    .yawSetWeight = 0.8f         
                }),
        m_turretCommand(drivers, m_turret, drivers.controlOperatorInterface),
        m_flywheels(drivers, 
            flywheel::m3508::FlywheelConfig {
            .leftFlyId = MotorId::MOTOR1, 
            .rightFlyId = MotorId::MOTOR2, 
            .canBus = CanBus::CAN_BUS2,
            .flyVelocityPidConfig = modm::Pid<float>::Parameter(180, 10, 0, 1000, 10000)
        }),      
        m_flywheelsCommand(m_flywheels, 0.036f),
        m_agitator(drivers,
                agitator::m3508::agitatorConfig{
                    .agitatorId = MotorId::MOTOR7,
                    .canBus = CanBus::CAN_BUS1,
                    .agitatorVelocityPidConfig = modm::Pid<float>::Parameter(180, 10, 0, 1000, 10000), 
                }),
        m_agitatorCommand(drivers, m_agitator, 45.5, &m_flywheelsCommand),
        m_wheelAgitator(drivers,
                agitator::m2006::agitatorConfig{
                    .agitatorId = MotorId::MOTOR3,
                    .canBus = CanBus::CAN_BUS2,
                    .agitatorVelocityPidConfig = modm::Pid<float>::Parameter(1000, 0, 0, 0, 16000), 
                }),
        m_wheelAgitatorCommand(drivers, m_wheelAgitator, 15.0, &m_flywheelsCommand)  
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
    m_wheelAgitator.initialize();
}

void Robot::registerSubsystems()
{
    m_drivers.commandScheduler.registerSubsystem(&m_chassis);
    m_drivers.commandScheduler.registerSubsystem(&m_turret);
    m_drivers.commandScheduler.registerSubsystem(&m_flywheels);
    m_drivers.commandScheduler.registerSubsystem(&m_agitator);
    m_drivers.commandScheduler.registerSubsystem(&m_wheelAgitator);
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

#endif  // TARGET_HERO