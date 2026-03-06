#pragma once

#include "flywheel_subsystem.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"
#include "tap/communication/gpio/pwm.hpp"

class Drivers;

namespace control::flywheel
{

class SnailFlywheelSubsystem : public FlywheelSubsystem
{
public:
    SnailFlywheelSubsystem(Drivers& drivers);
        
    ~SnailFlywheelSubsystem() = default;

    void initialize() override;

    void setTargetSpeed(float speed);

    void refresh() override;

    const char* getName() override { return "SnailFlywheel"; }

private:
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN1 = tap::gpio::Pwm::C1;
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN2 = tap::gpio::Pwm::C2;
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN3 = tap::gpio::Pwm::C3;
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN4 = tap::gpio::Pwm::C4;
    static constexpr float MAX_SNAIL_OUTPUT = 0.50f;    
    static constexpr float MIN_SNAIL_OUTPUT = 0.25f;
    static constexpr float OFF_PWM = 0.25f;    
    Drivers* m_drivers;

};

}  
