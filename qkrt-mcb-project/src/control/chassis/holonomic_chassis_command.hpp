#pragma once

#include <tap/control/command.hpp>

#include "control/chassis/holonomic_chassis_subsystem.hpp"
#include "communication/vision_coprocessor.hpp"
#include "control/control_operator_interface.hpp"

namespace control::turret {
    class TurretSubsystem;
}

namespace control::chassis
{

struct chassisCommandConfig {
    float maxChassisSpeed = 0.5f;
    float maxRotSpeed = 0.35f;
    float boostMultiplier = 1.3f;
};

class HolonomicChassisCommand : public tap::control::Command
{
public:
    HolonomicChassisCommand(Drivers &drivers,
                            HolonomicChassisSubsystem& chassis,
                            turret::TurretSubsystem& turret,
                            ControlOperatorInterface& m_operatorInterface, chassisCommandConfig config);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "Chassis Omni Drive Command"; }

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
    float m_boostMultiplier;
};

}  // namespace control::chassis