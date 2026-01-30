#pragma once

#include "drivers.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    #include "tap/mock/dji_motor_mock.hpp"
#else
    #include "tap/motor/dji_motor.hpp"
#endif

#include <tap/control/subsystem.hpp>
#include "tap/util_macros.hpp"

#include <array>
#include <cstdint>

namespace control::flywheel 
{
    struct FlywheelConfig
    {
        tap::motor::MotorId leftFlyWheelId;
        tap::motor::MotorId rightFlywheelId;
        tap::can::CanBus canBus;
        modm::Pid<float>::Parameter wheelVelocityPidConfig;
    };
    

class FlywheelSubsystem : public tap::control::Subsystem
{
private:
    enum class MotorId : uint8_t //change later 
    {
        LFly = 0,
        RFly,
        NUM_MOTORS,
    };

    static constexpr float MAX_WHEELSPEED_RPM = 5000.0f; //change maybe

    using Pid = modm::Pid<float>;
    using Motor = tap::motor::DjiMotor; //use a simplfied name for a complex datatype
public:
    FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config);

    void initialize() override;

    void setWheelVelocities(float flywheelSpeed);

    void refresh() override;

    const char* getName() override { return "Flywheel"; }

private: 
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_desiredOutput;
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_motors;
    std::array<Pid,   static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_pidControllers;
    Drivers* m_drivers;
};


}  // namespace control::flywheel