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

namespace control::agitator::m3508
{
struct agitatorConfig
{
    tap::motor::MotorId agitatorId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter agitatorVelocityPidConfig;
};

class M3508AgitatorSubsystem : public tap::control::Subsystem
{
private:

    using Pid = modm::Pid<float>;
    using Motor = tap::motor::DjiMotor;

    static constexpr float MAX_CURRENT = 16384.0f;
public:
    M3508AgitatorSubsystem(Drivers& drivers, const agitatorConfig& config);

    void initialize() override;

    void setAgitatorSpeed(float speed);
    
    /**
     * @brief Runs velocity PID controllers for the drive motors.
     */
    void refresh() override;

    const char* getName() const override { return "Agitator"; }
private:
    float m_desiredOutput;
    Pid m_pidController;
    Motor m_motor;
    Drivers* m_drivers;
};

}// namespace control::agitator::m3508