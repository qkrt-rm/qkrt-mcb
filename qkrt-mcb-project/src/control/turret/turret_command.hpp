#pragma once

#include <tap/control/command.hpp>

#include "control/control_operator_interface.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "communication/logger/logger.hpp"
#include "communication/vision_coprocessor.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/ballistics.hpp"

namespace control::turret
{

enum class SentryState
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

    // State Machine Variables
    SentryState m_currentState; 
    uint32_t m_targetLostTicks; 
    static constexpr uint32_t TARGET_LOST_TIMEOUT_TICKS = 500; 
    float m_scanDirection; 

    // Kalman Variables
    static constexpr int K_STATES = 6;
    static constexpr int K_INPUTS = 3;

    tap::algorithms::KalmanFilter<K_STATES, K_INPUTS> m_kalmanFilter;
    bool m_kalInit;

    modm::Vector3f m_currentRawPos;
    modm::Vector3f m_currentFilteredVel;
};

}  // namespace control::turret