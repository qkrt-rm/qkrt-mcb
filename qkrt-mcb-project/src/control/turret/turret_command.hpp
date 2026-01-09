#pragma once

#include <tap/control/command.hpp>

#include "control/control_operator_interface.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "communication/logger/logger.hpp"
#include "communication/vision_coprocessor.hpp"

namespace control::turret
{

class TurretCommand : public tap::control::Command
{
private:
    using Uart = tap::communication::serial::Uart;

public:
    TurretCommand(Drivers& drivers, TurretSubsystem& turret,
                  ControlOperatorInterface& m_operatorInterface);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Turret Command"; }

private:
    TurretSubsystem& m_turret;
    ControlOperatorInterface& m_operatorInterface;

    communication::VisionCoprocessor& m_visionCoprocessor;
    communication::logger::Logger& m_logger;

    bool isAutoAim;

    float m_pitchSensitivity, m_yawSensitivity;

    float m_globalYawTarget, m_globalPitchTarget;

    communication::TurretData m_lastTarget;
};

}  // namespace control::turret