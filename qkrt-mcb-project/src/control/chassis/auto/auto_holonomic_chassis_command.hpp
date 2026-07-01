#pragma once

#include <tap/control/command.hpp>

#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "control/chassis/holonomic_chassis_command.hpp"
#include "communication/vision_coprocessor.hpp"
#include "control/control_operator_interface.hpp"

namespace control::turret {
    class TurretSubsystem;
}

namespace control::chassis
{

class AutoHolonomicChassisCommand : public tap::control::Command
{
public:
    AutoHolonomicChassisCommand(Drivers &drivers,
                            HolonomicChassisSubsystem& chassis,
                            turret::TurretSubsystem& turret,
                            ControlOperatorInterface& m_operatorInterface, chassisCommandConfig config);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Chassis Omni Drive Command"; }

    bool isDriveLockTurret() {return islockTurret; }

    static constexpr float CHASSIS_ROT_SPEED_RAD = 0.35f;
private:
    HolonomicChassisSubsystem& m_chassis;
    turret::TurretSubsystem& m_turret;
    ControlOperatorInterface& m_operatorInterface;
    communication::VisionCoprocessor& m_visionCoprocessor;
    communication::logger::Logger& m_logger;
    Drivers* m_drivers;

    float m_maxSpeed;
    float m_chassisRotSpeed;
    float m_startTimer;
    bool isNavReady = false;
    float m_sequenceTimer = 0.0f;
    bool isHardCode = false; 
    bool islockTurret = true;
};

}  // namespace control::chassis