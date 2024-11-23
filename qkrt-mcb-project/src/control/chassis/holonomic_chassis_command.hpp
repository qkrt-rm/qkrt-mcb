#pragma once

#include <tap/control/command.hpp>

#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/control_operator_interface.hpp"

namespace control::chassis
{

/**
 * @brief Commands a ChassisSubsystem using tank drive. The left and right sides of the chassis are
 * commanded independently by this command. This command executes constantly, taking remote inputs
 * in from a control operator interface, transforming remote input into chassis speed.
 */
class HolonomicChassisCommand : public tap::control::Command
{
private:
    static constexpr float MAX_CHASSIS_SPEED_MPS = 1.0f;
public:

    /**
     * @brief Construct a new Chassis Tank Drive Command object
     *
     * @param chassis Chassis to control.
     */
    HolonomicChassisCommand(HolonomicChassisSubsystem& chassis, ControlOperatorInterface& operatorInterface);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Chassis Omni Drive Command"; }
private:
    HolonomicChassisSubsystem& _M_chassis;
    ControlOperatorInterface& _M_operatorInterface;
};

}  // namespace control::chassis