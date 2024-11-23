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


struct internal {
    static inline float turretYaw = 0.0f;
    static inline float turretPitch = 0.0f;
    static inline bool  indexerBoost = false;
};

namespace control
{
class ControlOperatorInterface;
}

namespace control::agitator
{
class VelocityAgitatorSubsystem;

class AgitatorCommand : public tap::control::Command
{
public:
    AgitatorCommand(VelocityAgitatorSubsystem &agitator, ControlOperatorInterface &operatorInterface, float indexerSpeed);

    const char *getName() const override { return "Chassis omni drive"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    VelocityAgitatorSubsystem &agitator;

    ControlOperatorInterface &operatorInterface;
    float indexerSpeed;
};

}  // namespace control::chassis