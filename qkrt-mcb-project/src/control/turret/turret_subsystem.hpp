#pragma once

#include "drivers.hpp"

#include <tap/control/subsystem.hpp>
#include "tap/motor/dji_motor.hpp"
#include "modm/math/filter/pid.hpp"

#include <array>

namespace control::turret
{

struct TurretConfig
{
    tap::motor::MotorId pitchId;
    tap::motor::MotorId yawId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter turretVelocityPidConfig;
};

class TurretSubsystem : public tap::control::Subsystem
{
private:
    enum class MotorId : uint8_t
    {
        PITCH = 0,
        YAW,
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;
    using Motor = tap::motor::DjiMotor;

public:
    TurretSubsystem(Drivers& drivers, const TurretConfig& config);

    void initialize() override;

    void adjustPitch(float angle);

    void adjustYaw(float angle);

    void refresh() override;

    const char* getName() override { return "Turret"; }

private:
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_desiredOutput;
    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_pidControllers;
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_motors;
};

};  // namespace control::turret