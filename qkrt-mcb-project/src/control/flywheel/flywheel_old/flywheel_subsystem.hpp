// /*
//  * Copyright (c) 2020-2021 Queen's Knights Robotics Team
//  *
//  * This file is part of qkrt-mcb.
//  *
//  * qkrt-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * qkrt-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with qkrt-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */

// #pragma once

// #include "tap/control/subsystem.hpp"
// #include "tap/util_macros.hpp"
// #include "tap/communication/gpio/pwm.hpp"

// class Drivers;

// namespace control
// {
// namespace flywheel
// {

// class FlywheelSubsystem : public tap::control::Subsystem
// {
// public:
//     FlywheelSubsystem(Drivers& drivers);
        
//     ~FlywheelSubsystem() = default;

//     void initialize() override;

//     void setDesiredOutput(float output);

//     void refresh() override;

// private:
//     static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN1 = tap::gpio::Pwm::C1;
//     static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN2 = tap::gpio::Pwm::C2;
//     static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN3 = tap::gpio::Pwm::C3;
//     static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN4 = tap::gpio::Pwm::C4;
//     static constexpr float MAX_SNAIL_OUTPUT = 0.50f;    
//     static constexpr float MIN_SNAIL_OUTPUT = 0.25f;
//     static constexpr float OFF_PWM = 0.25f;    
//     Drivers* m_drivers;

// };

// }  
// }  
