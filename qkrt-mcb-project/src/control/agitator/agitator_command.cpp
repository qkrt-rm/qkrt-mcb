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

#include "agitator_command.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/control_operator_interface.hpp"
#include "velocity_agitator_subsystem.hpp"

using tap::algorithms::limitVal;

namespace control::agitator
{
AgitatorCommand::AgitatorCommand(
    VelocityAgitatorSubsystem &agitator,
    ControlOperatorInterface &operatorInterface, float indexerSpeed)
    : m_agitator(agitator),
      m_operatorInterface(operatorInterface),
      m_indexerSpeed(indexerSpeed)
{
    addSubsystemRequirement(&agitator);
}

void AgitatorCommand::execute()
{
    m_operatorInterface.pollInputDevices();

    float newIndexerSpeed = internal::indexerBoost ? m_indexerSpeed  + 20.0f : m_indexerSpeed;

    if (m_operatorInterface.getAgitatorInput()) 
        m_agitator.setSetpoint(newIndexerSpeed);
    else 
        m_agitator.setSetpoint(0);
}

void AgitatorCommand::end(bool) { m_agitator.setSetpoint(0); }

};  // namespace control::chassis