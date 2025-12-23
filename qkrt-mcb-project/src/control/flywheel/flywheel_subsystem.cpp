/*
 * Copyright (c) 2020-2021 Queen's Knights Robotics Team
 *
 * This file is part of qkrt-mcb.
 *
 * qkrt-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * qkrt-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with qkrt-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "flywheel_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "drivers.hpp"

namespace control::flywheel
{
FlywheelSubsystem::FlywheelSubsystem(Drivers& drivers)
    : tap::control::Subsystem(&drivers)
{
}
        
void FlywheelSubsystem::initialize() { 
    drivers->pwm.write(0.25f, FLYWHEEL_MOTOR_PIN1);
    drivers->pwm.write(0.25f, FLYWHEEL_MOTOR_PIN2);
    drivers->pwm.write(0.25f, FLYWHEEL_MOTOR_PIN3);
    drivers->pwm.write(0.25f, FLYWHEEL_MOTOR_PIN4);
}

void FlywheelSubsystem::refresh() {}

void FlywheelSubsystem::setDesiredOutput(float output) {
    drivers->pwm.write(output, FLYWHEEL_MOTOR_PIN1);
    drivers->pwm.write(output, FLYWHEEL_MOTOR_PIN2);
    drivers->pwm.write(output, FLYWHEEL_MOTOR_PIN3);
    drivers->pwm.write(output, FLYWHEEL_MOTOR_PIN4);
    drivers->leds.set(tap::gpio::Leds::Green, true);
}

}  // control