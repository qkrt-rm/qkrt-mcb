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

    Drivers& m_drivers;


    /**
    * @brief Chassis subsystem for the sentry robot
    */
    chassis::HolonomicChassisSubsystem m_chassis;
    chassis::HolonomicChassisCommand m_chassisCommand;

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