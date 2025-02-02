#pragma once

#include <tap/control/command.hpp>

#include "control/control_operator_interface.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::turret
{

class TurretCommand : public tap::control::Command
{
private:
    using Uart = tap::communication::serial::Uart;

public:
    TurretCommand(TurretSubsystem& chassis,
                  ControlOperatorInterface& operatorInterface,
                  Uart& uart);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Turret Command"; }

private:
    TurretSubsystem& _M_turret;
    ControlOperatorInterface& _M_operatorInterface;
    Uart& _M_uart;

    float _M_pitchSensitivity, _M_yawSensitivity;
};

}  // namespace control::turret