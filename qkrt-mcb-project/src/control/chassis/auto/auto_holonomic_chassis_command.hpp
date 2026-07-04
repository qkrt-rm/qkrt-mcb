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


    float m_globalX = 0.0f;
    float m_globalY = 0.0f;
    float m_globalYaw = 0.0f;

    // Struct to hold absolute field coordinates of the tags
    struct TagLocation {
        float x;
        float y;
    };

    TagLocation getTagLocation(uint8_t id) {
        switch (id) {
            case 0: return {2.295f, 3.7f}; // Red Spawn (Example)
            case 1: return {2.33f,  2.72f}; // Red Middle Wall
            case 2: return {5.0f,  1.18f}; // Red Block
            case 3: return { 7.01f,  1.20f}; // Blue Block
            case 4: return { 9.62f,  2.68f}; // Blue Middle Wall
            case 5: return { 9.75f,  3.7f}; // Blue Spawn
            case 6: return {11.0f,  8.0f}; // Blue Ramp
            case 7: return { 5.97f,  1.7f}; // Middle Block
            case 8: return { 6.0f,  6.96f}; // Middle Ramp
            case 9: return {1.0f, 8.0f};  // Red Ramp
            default: return {0.0f,  0.0f};
        }
    }
};

}  // namespace control::chassis