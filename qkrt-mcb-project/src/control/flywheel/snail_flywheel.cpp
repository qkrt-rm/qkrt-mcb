#include "snail_flywheel.hpp"

#include "tap/communication/serial/remote.hpp"
#include "drivers.hpp"

using namespace control;

namespace control::flywheel
{
SnailFlywheelSubsystem::SnailFlywheelSubsystem(Drivers& drivers)
    : FlywheelSubsystem(drivers), 
    m_drivers(&drivers)
{
}
        
void SnailFlywheelSubsystem::initialize() { 
    drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN1);
    drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN2);
    drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN3);
    drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN4);
}

void SnailFlywheelSubsystem::refresh() {
    if (m_drivers->isEmergencyStopActive()) {
        m_drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN1);
        m_drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN2);
        m_drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN3);
        m_drivers->pwm.write(OFF_PWM, FLYWHEEL_MOTOR_PIN4);
    }
}

void SnailFlywheelSubsystem::setTargetSpeed(float speed) {
    drivers->pwm.write(speed, FLYWHEEL_MOTOR_PIN1);
    drivers->pwm.write(speed, FLYWHEEL_MOTOR_PIN2);
    drivers->pwm.write(speed, FLYWHEEL_MOTOR_PIN3);
    drivers->pwm.write(speed, FLYWHEEL_MOTOR_PIN4);
    drivers->leds.set(tap::gpio::Leds::Green, true);
}

}  // control