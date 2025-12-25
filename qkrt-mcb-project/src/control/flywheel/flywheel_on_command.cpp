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

#include "flywheel_on_command.hpp"

#include "tap/control/command.hpp"

#include "flywheel_subsystem.hpp"

#include "tap/communication/gpio/leds.hpp"

#include "control/control_operator_interface.hpp"

namespace control
{
namespace flywheel
{
void FlywheelOnCommand::initialize() {}

void FlywheelOnCommand::execute() 
{    
    m_operatorInterface.pollInputDevices();

    if (m_operatorInterface.getFlywheelInput())
        m_flywheel.setDesiredOutput(m_flywheelPWM);
    else
        m_flywheel.setDesiredOutput(OFF_PWM); 
}

void FlywheelOnCommand::end(bool) { m_flywheel.setDesiredOutput(OFF_PWM); }

bool FlywheelOnCommand::isFinished() const { return false; }
}  // namespace flywheel
}  // namespace control
