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