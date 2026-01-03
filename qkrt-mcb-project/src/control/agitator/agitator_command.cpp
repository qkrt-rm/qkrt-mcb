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
    VelocityAgitatorSubsystem &agitator, float indexerSpeed)
    : m_agitator(agitator),
      m_indexerSpeed(indexerSpeed)
{
    addSubsystemRequirement(&agitator);
}

void AgitatorCommand::execute()
{
    /**
     * TODO:
     * - Jammed Timer
     * - Barrel Overheat Limiting
     * - Use Balls Per Second instead of rpm
     * - UNIT FIX reading radians per second now
     */

    float newIndexerSpeed = isBOOST ? m_indexerSpeed  + 20.0f : m_indexerSpeed;
    m_agitator.setSetpoint(newIndexerSpeed);
}

void AgitatorCommand::end(bool) { m_agitator.setSetpoint(0); }

};  // namespace control::chassis