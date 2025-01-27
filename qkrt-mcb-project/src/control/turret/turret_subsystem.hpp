#pragma once

#include "drivers.hpp"

#include <tap/control/subsystem.hpp>
#include "tap/util_macros.hpp"

#include "tap/motor/dji_motor.hpp"
#include "modm/math/filter/pid.hpp"

#include <array>

#include "math/vector.hpp"

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

    static constexpr float MAX_TURRET_MOTOR_RPM        = 320.0f;
    static constexpr float MAX_TURRET_MOTOR_MILLIVOLTS = 25'000.0f;

    static constexpr float MAX_TURRET_ELEVATION =  M_PI_4;

public:
    TurretSubsystem(Drivers& drivers, const TurretConfig& config);

    void initialize() override;
    void refresh() override;
    const char* getName() override { return "Turret"; }

public:
    void setPitch(float elevation);
    void setYaw(float azimuth);
    
private:
    inline float rpsToRpm(float radPerSec) const
    {
        static constexpr float RAD_PER_REV = M_TWOPI;
        static constexpr float GEAR_RATIO  = 1.0f;

        return radPerSec / RAD_PER_REV * GEAR_RATIO;
    }

    inline float rpmToMilliVolts(float revPerMin) const
    {
        return revPerMin * MAX_TURRET_MOTOR_MILLIVOLTS / MAX_TURRET_MOTOR_RPM;
    }

    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_desiredOutput;
    std::array<Pid,   static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_pidControllers;
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_motors;

    float _M_elevation, _M_azimuth;
    float _M_sensitivity;

    int64_t _M_yawForwardOffset;
};

};  // namespace control::turret