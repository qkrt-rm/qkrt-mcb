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
        LF = 0,  ///< Left front
        LB,      ///< Left back
        RF,      ///< Right front
        RB,      ///< Right back
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    static constexpr float MAX_WHEELSPEED_RPM = 7000.0f;
public:
    HolonomicChassisSubsystem(Drivers& drivers, const ChassisConfig& config);

    void initialize() override;

    ///
    /// @brief Sets the wheel velocity of the four drive motors individually based on 
    /// the input desired velocities.
    ///
    /// @param leftFront Desired speed in m/s of the left-front wheel of the chassis.
    /// @param leftBack Desired speed in m/s of the left-back wheel of the chassis.
    /// @param rightFront Desired chassis speed in m/s of the right-front of the chassis.
    /// @param rightBack Desired speed in m/s of the right-back wheel of the chassis.
    ///
    void setWheelVelocities(float leftFront, float leftBack, float rightFront, float rightBack);
    
    ///
    /// @brief Runs velocity PID controllers for the drive motors.
    ///
    void refresh() override;

    const char* getName() override { return "Chassis"; }
private:
    inline float mpsToRpm(float mps) const
    {
        static constexpr float GEAR_RATIO = 19.0f;
        static constexpr float WHEEL_DIAMETER_M = 0.076f;
        static constexpr float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
        static constexpr float SEC_PER_M = 60.0f;

        return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
    }

    /// Desired wheel output for each motor
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_desiredOutput;
    /// PID controllers. Input desired wheel velocity, output desired motor current.
    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_pidControllers;
protected:
    /// Motors.
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> _M_motors;
};

}  // namespace control::chassis