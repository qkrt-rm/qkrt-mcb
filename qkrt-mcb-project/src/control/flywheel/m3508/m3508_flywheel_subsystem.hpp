#pragma once

#include "drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    #include "tap/mock/dji_motor_mock.hpp"
#else
    #include "tap/motor/dji_motor.hpp"
#endif

#include <tap/control/subsystem.hpp>
#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"

#include "control/constants.hpp"
#include <array>

namespace control::flywheel::m3508
{
struct FlywheelConfig
{
    tap::motor::MotorId leftFlyId;
    tap::motor::MotorId rightFlyId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter flyVelocityPidConfig;
};

class M3508FlywheelSubsystem : public tap::control::Subsystem
{
private:

    enum class MotorId : uint8_t //change later 
    {
        LFly = 0,
        RFly,
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;
    using Motor = tap::motor::DjiMotor;

    static constexpr float MAX_CURRENT = 16384.0f;
public:
    M3508FlywheelSubsystem(Drivers& drivers, const FlywheelConfig& config);

    void initialize() override;


    void setFlywheelVel(float speed);
    
    /**
     * @brief Runs velocity PID controllers for the drive motors.
     */
    void refresh() override;

    const char* getName() const override { return "Flyhweel"; }
private:
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_desiredOutput;
    std::array<Pid,   static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_pidControllers;
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_motors;
    Drivers* m_drivers;
};


}  // namespace control::flywheel::m3508