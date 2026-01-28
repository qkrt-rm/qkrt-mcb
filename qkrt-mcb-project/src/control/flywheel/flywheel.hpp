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

namespace control::flywheel 
{
    struct FlywheelConfig
    {
        tap::motor::MotorId leftFlyWheelId;
        tap::motor::MotorId rightFlywheelId;
        tap::can::CanBus canBus;
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

    static constexpr float MAX_WHEELSPEED_RPM = 3000.0f; //change maybe

    using Motor = tap::motor::DjiMotor; //use a simplfied name for a complex datatype
public:
    FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config);

    void initialize() override;

    void setDesiredOuput(float flywheelSpeed);

    void refresh() override;

    const char* getName() override {return "Flywheel";}

private: 
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_desiredOutput;
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_motors;
    Drivers* m_drivers;
};


}  // namespace control::flywheel
