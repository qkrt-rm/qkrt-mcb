#pragma once


// Taproot Drivers Include
#include "drivers.hpp"

//Hardware Constants
#include "control/constants.hpp"

// Chassis (Holonomic Drive) Includes
#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/chassis/auto/auto_holonomic_chassis_command.hpp"


//Turret Includes
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/auto/auto_turret_command.hpp"


// Flywheel Includes
#include "control/flywheel/snail/snail_flywheel_subsystem.hpp"
#include "control/flywheel/snail/snail_flywheel_on_command.hpp"


// Agitator Includes
#include "control/agitator/M2006/m2006_velocity_agitator_subsystem.hpp"
#include "control/agitator/M2006/m2006_agitator_command.hpp"

// currently unknown:
#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"

using tap::can::CanBus;
using tap::motor::MotorId;

namespace control
{
class Robot
{
public:
    Robot(Drivers& drivers);

    void initialize();
private:
    void initializeSubsystems();
    void registerSubsystems();
    void setDefaultCommands();
    void startCommands();
    void registerIoMappings();
private:

    chassis::ChassisConfig m_chassisConfig {
        .leftFrontId  = MotorId::MOTOR1,
        .leftBackId   = MotorId::MOTOR2,
        .rightBackId  = MotorId::MOTOR3,
        .rightFrontId = MotorId::MOTOR4,
        .canBus       = CanBus::CAN_BUS1,
        .wheelVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000), // TODO: tune this
    };

    chassis::chassisCommandConfig m_chassisCmdConfig {
        .maxChassisSpeed = 0.6f,
        .maxRotSpeed = 0.70f,
    };

    turret::TurretConfig turretConfig {
        .pitchId = MotorId::MOTOR6,
        .yawId   = MotorId::MOTOR5,
        .pitchGearRatio = GM6020::GEAR_RATIO,
        .yawGearRatio = GM6020::GEAR_RATIO,
        .pitchInverted = false,
        .yawInverted = true,
        .isYawZeroed = true,
        .mcbHoriz = true,
        .canBus  = CanBus::CAN_BUS1,
        .yawForwardOffset = -5.242f,
        .pitchHorizontalOffset = -1.91901f, 
        .pitchUpLim = 0.07,
        .pitchDownLim = -0.155,
        .MAX_PITCH_POWER = GM6020::MAX_VOLTAGE,
        .MAX_YAW_POWER = GM6020::MAX_VOLTAGE,
        .MAX_RPS = GM6020::MAX_RPS,
        .pitchPosGains = { .kp = 10.5f, .ki = 0.2f, .kd = 0.0f, .maxICumulative = 500.0f, .maxOutput = GM6020::MAX_VOLTAGE },        
        .pitchVelGains = { .kp = 6000.0f, .ki = 110.0f, .kd = 0.0f, .maxICumulative = 3000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
        .yawPosGains   = { .kp = 6.5f,  .ki = 0.0f, .kd = 1.15f, .maxICumulative = 5000.0f, .maxOutput = GM6020::MAX_VOLTAGE},        
        .yawVelGains   = { .kp = 8000.0f, .ki = 10.0f,  .kd = 0.0f, .maxICumulative = 1000.0f, .maxOutput = GM6020::MAX_VOLTAGE},
        .yawFF = 4618.0f,
        .pitchFF = 1000.0f
    };

    float m_flywheelSpeed = 0.38f;
    float m_agitatorSpeed = -3.5f;

    agitator::m2006::agitatorConfig m_agitatorConfig {
        .agitatorId = MotorId::MOTOR7,
        .canBus = CanBus::CAN_BUS1,
        .agitatorVelocityPidConfig = modm::Pid<float>::Parameter(1700, 0, 0, 0, 16000), 
    };

    Drivers& m_drivers;

    /**
    * @brief Chassis subsystem for the sentry robot
    */
    chassis::HolonomicChassisSubsystem m_chassis;
    chassis::AutoHolonomicChassisCommand m_chassisCommand;

    /**
     * @brief Turret subsystem for the sentry robot
     */
    turret::TurretSubsystem m_turret;
    turret::AutoTurretCommand m_turretCommand;


    /**
     * @brief Flywheel subsystem for the sentry robot
     */
    flywheel::snail::SnailFlywheelSubsystem m_flywheels;
    flywheel::snail::SnailFlywheelOnCommand m_flywheelsCommand;


    /**
     * @brief Agitator subsystem for the sentry robot
     */
    agitator::m2006::VelocityAgitatorSubsystem m_agitator;
    agitator::m2006::AgitatorCommand m_agitatorCommand;


    //Mappings
    tap::control::HoldCommandMapping m_leftSwitchUP{
        &m_drivers,
        {&m_agitatorCommand},
        tap::control::RemoteMapState(tap::communication::serial::Remote::Switch::LEFT_SWITCH, 
            tap::communication::serial::Remote::SwitchState::UP
        )
    };

    tap::control::HoldCommandMapping m_rightSwitchUP{
    &m_drivers,
    {&m_flywheelsCommand},
    tap::control::RemoteMapState(tap::communication::serial::Remote::Switch::RIGHT_SWITCH, 
        tap::communication::serial::Remote::SwitchState::UP
    )
    };

    tap::control::HoldCommandMapping m_rightMouseFlywheel{
        &m_drivers,
        {&m_flywheelsCommand},
        tap::control::RemoteMapState(tap::control::RemoteMapState::MouseButton::RIGHT)
    };

    tap::control::HoldCommandMapping m_leftMouseIndex{
        &m_drivers,
        {&m_agitatorCommand},
        tap::control::RemoteMapState(tap::control::RemoteMapState::MouseButton::LEFT)
    };

};
  
}  // namespace control