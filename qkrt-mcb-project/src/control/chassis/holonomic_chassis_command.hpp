#pragma once

#include <tap/control/command.hpp>

#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/control_operator_interface.hpp"

#include "control/turret/turret_subsystem.hpp"

namespace control::chassis
{

class HolonomicChassisCommand : public tap::control::Command
{
private:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 1.0f;
public:
    HolonomicChassisCommand(HolonomicChassisSubsystem& chassis,
                            turret::TurretSubsystem& turret,
                            ControlOperatorInterface& operatorInterface,
                            tap::communication::sensors::imu::bmi088::Bmi088& imu
                            );

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Chassis Omni Drive Command"; }
private:
    HolonomicChassisSubsystem& _M_chassis;
    turret::TurretSubsystem& _M_turret;
    ControlOperatorInterface& _M_operatorInterface;
    tap::communication::sensors::imu::bmi088::Bmi088& _M_imu;
};

}  // namespace control::chassis