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
                  ControlOperatorInterface& m_operatorInterface,
                  Uart& uart);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Turret Command"; }

private:
    TurretSubsystem& m_turret;
    ControlOperatorInterface& m_operatorInterface;
    Uart& m_uart;

    float m_pitchSensitivity, m_yawSensitivity;

    // TODO: change to some target data structure when it exists
    void* m_target;
};

}  // namespace control::turret