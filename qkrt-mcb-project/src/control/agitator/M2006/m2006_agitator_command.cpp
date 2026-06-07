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

#include "m2006_agitator_command.hpp"

using tap::algorithms::limitVal;

namespace control::agitator::m2006
{
AgitatorCommand::AgitatorCommand(
    Drivers &drivers, VelocityAgitatorSubsystem &agitator, float indexerSpeed,
    tap::control::Command* flywheelsCommand)
    :   m_drivers(&drivers),
        m_agitator(agitator),
        m_indexerSpeed(indexerSpeed),
        m_flywheelsCommand(flywheelsCommand)
{
    addSubsystemRequirement(&agitator);
}

bool AgitatorCommand::isReady() 
{
    //check flywheels are on before agitator command
    return m_drivers->commandScheduler.isCommandScheduled(m_flywheelsCommand);
}

bool AgitatorCommand::isFinished() const
{
    //trigger end command when flywheels are off
    return !m_drivers->commandScheduler.isCommandScheduled(m_flywheelsCommand);
}

void AgitatorCommand::execute()
{
    /**
     * TODO:
     * - Barrel Overheat Limiting
     * - Use Balls Per Second instead of rpm
     */

     m_agitator.setSetpoint(m_indexerSpeed);
}

void AgitatorCommand::end(bool) 
{ 
    m_agitator.setSetpoint(0.0f); 
}

};  // namespace control::agitator::m2006