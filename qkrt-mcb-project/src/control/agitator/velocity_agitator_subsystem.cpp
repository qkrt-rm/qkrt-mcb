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

#include "velocity_agitator_subsystem.hpp"

#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"

#include "drivers.hpp"

using tap::arch::clock::getTimeMilliseconds;
using tap::motor::DjiMotor;

namespace control::agitator
{
VelocityAgitatorSubsystem::VelocityAgitatorSubsystem(
    Drivers& drivers,
    const control::algorithms::EduPidConfig& pidConfig,
    tap::motor::DjiMotor& agitator)
    : Subsystem(&drivers),
      agitator(agitator),
      velocityPid(pidConfig)
{
}

void VelocityAgitatorSubsystem::initialize() { agitator.initialize(); }

void VelocityAgitatorSubsystem::refresh() {
    if(!isOnline()){
        calibrated = false;
    }
    if(calibrated){
        velocityPid.runControllerDerivateError(getSetpoint() - getCurrentValue(), tap::arch::clock::getTimeMilliseconds() - prevTime);
        prevTime = tap::arch::clock::getTimeMilliseconds();
        agitator.setDesiredOutput(velocityPid.getOutput());
    }else{
        calibrateHere();
    }
}

float VelocityAgitatorSubsystem::getSetpoint() const {
    return velocitySetpoint;
}

float VelocityAgitatorSubsystem::getCurrentValue() const {
    
    return agitator.getShaftRPM() / AGITATOR_GEAR_RATIO_M2006 * (M_TWOPI / 60);
}

bool VelocityAgitatorSubsystem::calibrateHere() {
    if(isOnline()){
        agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
        calibrated = true;
        return true;
    }else{
        return false;
    }
}

bool VelocityAgitatorSubsystem::isOnline() {
    return agitator.isMotorOnline();
}

float VelocityAgitatorSubsystem::getCurrentValueIntegral() const {
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
}

float VelocityAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    return (2.0f * M_PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           agitator.getEncoderUnwrapped() / AGITATOR_GEAR_RATIO_M2006;
}
}  // namespace control::agitator
