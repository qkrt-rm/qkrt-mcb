#pragma once

#include <tap/control/command.hpp>

#include "control/turret/turret_subsystem.hpp"
#include "control/control_operator_interface.hpp"
#include "communication/serial/logger.hpp"

namespace control::turret
{

class TurretCommand : public tap::control::Command
{
private:
    using TerminalSerial = tap::communication::serial::TerminalSerial;

public:
    TurretCommand(Drivers& drivers, TurretSubsystem& chassis);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Turret Command"; }

private:
    TurretSubsystem& _M_turret;
    ControlOperatorInterface& _M_operatorInterface;
    communication::serial::Logger & _M_logger;

    float _M_pitchSensitivity, _M_yawSensitivity;

    // TODO: change to some target data structure when it exists
    void* _M_target;
};

}  // namespace control::turret