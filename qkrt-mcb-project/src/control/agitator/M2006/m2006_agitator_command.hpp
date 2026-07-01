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

#pragma once

#include "tap/control/command.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "control/control_operator_interface.hpp"
#include "m2006_velocity_agitator_subsystem.hpp"
#include "drivers.hpp"

namespace control::agitator::m2006
{
class VelocityAgitatorSubsystem;

class AgitatorCommand : public tap::control::Command
{
public:
    AgitatorCommand(Drivers& drivers, VelocityAgitatorSubsystem &agitator, float indexerSpeed, bool isHeroWheel, 
        tap::control::Command* flywheelsCommand, ControlOperatorInterface& operatorInterface);

    const char *getName() const override { return "agitator command"; }

    bool isReady() override;
    void initialize() override {}
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
        
    Drivers* m_drivers;
    VelocityAgitatorSubsystem &m_agitator;
    float m_indexerSpeed;
    tap::control::Command* m_flywheelsCommand;
    ControlOperatorInterface& m_operatorInterface;
    bool m_isCtrlReversal;

};

}  // namespace control::agitator::m2006