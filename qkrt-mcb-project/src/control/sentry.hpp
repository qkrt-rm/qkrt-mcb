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

    Drivers& _M_drivers;


    /**
    * @brief Chassis subsystem for the sentry robot
    */
    chassis::HolonomicChassisSubsystem _M_chassis;
    chassis::HolonomicChassisCommand _M_chassisCommand;

    /**
     * @brief Turret subsystem for the sentry robot
     */
    turret::TurretSubsystem _M_turret;
    turret::TurretCommand _M_turretCommand;


    /**
     * @brief Flywheel subsystem for the sentry robot
     */
    flywheel::FlywheelSubsystem _M_flywheels;
    flywheel::FlywheelOnCommand _M_flywheelsCommand;


    /**
     * @brief Agiator subsystem for the sentry robot
     */
    tap::motor::DjiMotor _M_agitator;
    agitator::VelocityAgitatorSubsystem _M_velocityAgitatorSubsystem;
    agitator::AgitatorCommand _M_agitatorCommand;

    algorithms::EduPidConfig eduPidConfig; 
    tap::control::setpoint::MoveIntegralCommand::Config moveIntegralConfig;
    tap::control::setpoint::MoveIntegralCommand moveIntegralCommand;
};
  
}  // namespace control