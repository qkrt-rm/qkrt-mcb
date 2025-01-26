#pragma once

#include <tap/control/command.hpp>

#include "control/control_operator_interface.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::turret
{

class TurretCommand : public tap::control::Command
{
private:

public:
    TurretCommand(TurretSubsystem& chassis, ControlOperatorInterface& operatorInterface);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Turret Command"; }

private:
    TurretSubsystem& _M_turret;
    ControlOperatorInterface& _M_operatorInterface;

};

}  // namespace control::turret