#pragma once

#include <tap/control/command.hpp>

#include "control/control_operator_interface.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "communication/logger/logger.hpp"
#include "communication/vision_coprocessor.hpp"
#include "tap/communication/gpio/digital.hpp"

namespace control::turret
{

enum class SentryState //THE TWO STATES OF THE TURRET, SCANNING FOR TARGETS AND SHOOTING AT TARGETS
{
    SCANNING,
    SHOOTING
};

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

    Drivers &m_drivers;
    
    bool isAutoAim;

    float m_globalPitch, m_globalYaw;

    float m_pitchCommand, m_yawCommand;

    float m_pitchSensitivity, m_yawSensitivity;

    float m_globalYawTarget, m_globalPitchTarget;

    communication::TurretData m_lastTarget;

    SentryState m_currentState; /*= SentryState::SCANNING;*/ //START IN SCANNING STATE

    uint32_t m_targetLostTicks; //IF THE TURRET LOSES SIGHT OF THE TARGET, TIMER DETERMINES IF REVERT TO SCANNING STATE. TIMER REPLACED WITH BETTER METHOD

    // 500 ticks at 2ms (TurretSubsystem::DT) = 1 second of grace period
    static constexpr uint32_t TARGET_LOST_TIMEOUT_TICKS = 500; //NUMBER OF TICKS BEFORE SWITCHING BACK TO SCANNING STATE, TUNE THIS BASED ON HOW LONG IT TAKES FOR THE VISION COPROCESSOR TO UPDATE TARGET DATA
    
    float m_scanDirection; //-pi/2 to pi/2, DETERMINES DIRECTION OF SCANNING, SWITCHES EVERY TIME THE TURRET HITS A SCAN LIMIT

};

}  // namespace control::turret