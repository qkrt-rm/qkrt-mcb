#pragma once


// Taproot Drivers Include
#include "drivers.hpp"

// Chassis (Holonomic Drive) Includes
#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/chassis/holonomic_chassis_command.hpp"


//Turret Includes
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/turret_command.hpp"


// Flywheel Includes
#include "control/flywheel/flywheel_subsystem.hpp"
#include "control/flywheel/flywheel_on_command.hpp"


// Agitator Includes
#include "control/agitator/velocity_agitator_subsystem.hpp"
#include "control/agitator/agitator_command.hpp"

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
    turret::TurretCommand m_turretCommand;


    /**
     * @brief Flywheel subsystem for the sentry robot
     */
    flywheel::FlywheelSubsystem m_flywheels;
    flywheel::FlywheelOnCommand m_flywheelsCommand;


    /**
     * @brief Agiator subsystem for the sentry robot
     */
    agitator::VelocityAgitatorSubsystem m_agitator;
    agitator::AgitatorCommand m_agitatorCommand;

    //Mappings
    tap::control::HoldCommandMapping m_leftSwitchUP{
        &m_drivers,
        {&m_agitatorCommand, &m_flywheelsCommand},
        tap::control::RemoteMapState(tap::communication::serial::Remote::Switch::LEFT_SWITCH, 
            tap::communication::serial::Remote::SwitchState::UP
        )
    };

    //TODO: Add keyboard mapping
};
  
}  // namespace control