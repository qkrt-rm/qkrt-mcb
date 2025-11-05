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
    TurretCommand(Drivers& drivers, TurretSubsystem& chassis,
                  ControlOperatorInterface& operatorInterface);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Turret Command"; }

private:
    TurretSubsystem& _M_turret;
    ControlOperatorInterface& _M_operatorInterface;

    float _M_pitchSensitivity, _M_yawSensitivity;

    communication::serial:: Logger & _M_Logger2;

    // TODO: change to some target data structure when it exists
    void* _M_target;
};

}  // namespace control::turret