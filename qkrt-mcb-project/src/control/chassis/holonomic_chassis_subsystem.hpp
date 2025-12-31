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

#include <array>

namespace control::chassis
{
struct ChassisConfig
{
    tap::motor::MotorId leftFrontId;
    tap::motor::MotorId leftBackId;
    tap::motor::MotorId rightBackId;
    tap::motor::MotorId rightFrontId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter wheelVelocityPidConfig;
};

class HolonomicChassisSubsystem : public tap::control::Subsystem
{
private:
    enum class MotorId : uint8_t
    {
        LF = 0,  // left-front
        LB,      // left-back
        RB,      // right-back
        RF,      // right-front
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;
    using Motor = tap::motor::DjiMotor;

    static constexpr float MAX_WHEELSPEED_RPM = 3000.0f;
public:
    HolonomicChassisSubsystem(Drivers& drivers, const ChassisConfig& config);

    void initialize() override;

    /**
     * @brief Sets the wheel velocity of the four drive motors individually based on 
     * the input desired velocities.
     *
     * @param leftFront Desired speed in m/s of the left-front wheel of the chassis.
     * @param leftBack Desired speed in m/s of the left-back wheel of the chassis.
     * @param rightBack Desired speed in m/s of the right-back wheel of the chassis.
     * @param rightFront Desired chassis speed in m/s of the right-front of the chassis.
     */
    void setWheelVelocities(float leftFront, float leftBack, float rightBack, float rightFront);
    
    /**
     * @brief Runs velocity PID controllers for the drive motors.
     */
    void refresh() override;

    const char* getName() override { return "Chassis"; }
private:
    inline float mpsToRpm(float mps) const
    {
        static constexpr float SEC_PER_MIN = 60.0f;
        static constexpr float GEAR_RATIO = 19.0f;
        static constexpr float WHEEL_DIAMETER_M = 0.076f;
        static constexpr float WHEEL_CIRCUMFERENCE_M = M_PI * WHEEL_DIAMETER_M;

        return (mps / WHEEL_CIRCUMFERENCE_M) * SEC_PER_MIN * GEAR_RATIO;
    }

    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_desiredOutput;
    std::array<Pid,   static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_pidControllers;
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_motors;
};


}  // namespace control::chassis