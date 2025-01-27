#pragma once

#include "drivers.hpp"

#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/chassis/holonomic_chassis_command.hpp"

#include "control/turret/turret_subsystem.hpp"
#include "control/turret/turret_command.hpp"

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

    chassis::HolonomicChassisSubsystem _M_chassis;
    chassis::HolonomicChassisCommand _M_chassisCommand;

    turret::TurretSubsystem _M_turret;
    turret::TurretCommand _M_turretCommand;
};
  
}  // namespace control