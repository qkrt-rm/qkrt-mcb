#pragma once

#include <tap/control/command.hpp>

#include "control/control_operator_interface.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "communication/logger/logger.hpp"
#include "communication/vision_coprocessor.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/extended_kalman.hpp"

namespace control::turret
{

enum class SentryState
{
    SCANNING,
    SHOOTING
};

class AutoTurretCommand : public tap::control::Command
{
private:
    using Uart = tap::communication::serial::Uart;

public:
    AutoTurretCommand(Drivers& drivers, TurretSubsystem& turret,
                  ControlOperatorInterface& m_operatorInterface,
                  tap::control::Command* flywheelsCommand, ///
                  tap::control::Command* agitatorCommand); ///

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
    float m_pitchBoresightTrim;
    float m_yawBoresightTrim;

    communication::TurretData m_lastTarget;

    // State Machine Variables
    SentryState m_currentState; 
    uint32_t m_targetLostTicks; 
    uint32_t m_targetStartTicks;
    static constexpr uint32_t TARGET_LOST_TIMEOUT_TICKS = 500; 
    static constexpr uint32_t TARGET_START_SHOOTING_TICKS = 100; 
    static constexpr uint32_t TARGET_ACQUIRE_TICKS = 5;
    uint32_t m_targetAcquireTicks;
    float m_scanDirection; 

    tap::control::Command* m_flywheelsCommand; ///
    tap::control::Command* m_agitatorCommand; ///

    static constexpr float MAX_PITCH_STEP_PER_UPDATE = 0.03f;

    // Kalman Variables
    static constexpr int K_STATES = 6;
    static constexpr int K_INPUTS = 3;

    tap::algorithms::KalmanFilter<K_STATES, K_INPUTS> m_kalmanFilter;
    bool m_kalInit;

    modm::Vector3f m_currentRawPos;
    modm::Vector3f m_currentFilteredVel;

    tap::algorithms::ExtendedKalman m_pitchFilter;
};

}  // namespace control::turret