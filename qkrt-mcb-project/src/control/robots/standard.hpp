#pragma once


// Taproot Drivers Include
#include "drivers.hpp"

//Hardware Constants
#include "control/constants.hpp"

// Chassis (Holonomic Drive) Includes
#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/chassis/holonomic_chassis_command.hpp"

//Turret Includes
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/turret_command.hpp"

// Flywheel Includes
#include "control/flywheel/m3508/m3508_flywheel_subsystem.hpp"
#include "control/flywheel/m3508/m3508_flywheel_on_command.hpp"

// Agitator Includes
#include "control/agitator/M2006/m2006_velocity_agitator_subsystem.hpp"
#include "control/agitator/M2006/m2006_agitator_command.hpp"

// currently unknown:
#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include <tap/control/toggle_command_mapping.hpp>

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

    chassis::chassisCommandConfig m_chassisCommandConfig {
        .maxChassisSpeed = 0.73f,     //POWER PROFILE
        //.maxChassisSpeed = 0.61f,       //HP PROFILE
        .maxRotSpeed = 0.70f
    };

    turret::TurretConfig m_turretConfig {
        .pitchId = MotorId::MOTOR6,
        .yawId   = MotorId::MOTOR5,
        .pitchGearRatio = GM6020::GEAR_RATIO,
        .yawGearRatio = GM6020::GEAR_RATIO,
        .pitchInverted = true,
        .yawInverted = true,
        .isYawZeroed = true,
        .mcbHoriz = false,
        .canBus  = CanBus::CAN_BUS1,
        .yawForwardOffset = -4.7155f,          
        .pitchHorizontalOffset = -3.1523f,      
        .pitchUpLim = 0.6200f,                  
        .pitchDownLim = -0.3275f,               
        .MAX_PITCH_POWER = GM6020::MAX_VOLTAGE,
        .MAX_YAW_POWER = GM6020::MAX_VOLTAGE,
        .MAX_RPS = GM6020::MAX_RPS,
        .pitchPosGains = { .kp = 10.8f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 500.0f, .maxOutput = GM6020::MAX_VOLTAGE },
        .pitchVelGains = { .kp = 4500.0f, .ki = 110.0f, .kd = 0.0f, .maxICumulative = 3000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
        .yawPosGains   = { .kp = 5.0f,  .ki = 0.0f, .kd = 0.0f, .maxICumulative = 5000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
        .yawVelGains   = { .kp = 9800.0f, .ki = 100.0f,  .kd = 0.0f, .maxICumulative = 1000.0f, .maxOutput = GM6020::MAX_VOLTAGE },
        .yawFF = 4618.0f,
        .pitchFF = 1000.0f
    };

    float m_flyhweelSpeed = 0.0395f;
    float m_agitatorSpeed = -8.5f;

    flywheel::m3508::FlywheelConfig m_flywheelConfig {
        .leftFlyId = MotorId::MOTOR1, 
        .rightFlyId = MotorId::MOTOR2, 
        .canBus = CanBus::CAN_BUS2,
        .flyVelocityPidConfig = modm::Pid<float>::Parameter(15, 1, 0, 1000, 10000)
    };

    agitator::m2006::agitatorConfig m_agitatorConfig {
        .agitatorId = MotorId::MOTOR3,
        .canBus = CanBus::CAN_BUS2,
        .agitatorVelocityPidConfig = modm::Pid<float>::Parameter(1000, 0, 0, 0, 16000), 
    };

    Drivers& m_drivers;

    /**
    * @brief Chassis subsystem for the standard robot
    */
    chassis::HolonomicChassisSubsystem m_chassis;
    chassis::HolonomicChassisCommand m_chassisCommand;

    /**
     * @brief Turret subsystem for the standard robot
     */
    turret::TurretSubsystem m_turret;
    turret::TurretCommand m_turretCommand;


    /**
     * @brief Flywheel subsystem for the standard robot
     */
    flywheel::m3508::M3508FlywheelSubsystem m_flywheels;
    flywheel::m3508::M3508FlywheelOnCommand m_flywheelsCommand;


    /**
     * @brief Agitator subsystem for the standard robot
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

    tap::control::ToggleCommandMapping m_ToggleFlyX{
        &m_drivers,
        {&m_flywheelsCommand},
        tap::control::RemoteMapState(
        std::list<tap::communication::serial::Remote::Key>{
            tap::communication::serial::Remote::Key::X
        })
    };

    tap::control::HoldCommandMapping m_leftMouseIndex{
        &m_drivers,
        {&m_agitatorCommand},
        tap::control::RemoteMapState(tap::control::RemoteMapState::MouseButton::LEFT)
    };

};
  
}  // namespace control