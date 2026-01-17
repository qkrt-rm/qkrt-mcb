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
using Motor = tap::motor::DjiMotor;

namespace control::agitator
{
VelocityAgitatorSubsystem::VelocityAgitatorSubsystem(Drivers& drivers, const agitatorConfig &config)
    : Subsystem(&drivers), 
      m_agitator(&drivers, config.agitatorId, config.canBus, false, "VA"),
      m_drivers(&drivers)
    {
        m_velocityPid.setParameter(config.agitatorVelocityPidConfig);
    }

void VelocityAgitatorSubsystem::initialize() { m_agitator.initialize(); }

void VelocityAgitatorSubsystem::refresh() {
    if(!isOnline()){
        calibrated = false;
    }
    if(calibrated){
        if (m_drivers->isEmergencyStopActive()) {
            m_velocityPid.reset();
            m_velocityPid.update(0.0f);
        }
        else {
            m_velocityPid.update(getSetpoint() - getCurrentValue());
        }
        m_agitator.setDesiredOutput(m_velocityPid.getValue());
    }else
    {
        calibrateHere();
    }
}

float VelocityAgitatorSubsystem::getSetpoint() const {
    return velocitySetpoint;
}

float VelocityAgitatorSubsystem::getCurrentValue() const {
    
    return m_agitator.getEncoder()->getVelocity() / AGITATOR_GEAR_RATIO_M2006 * (M_TWOPI / 60);
}

bool VelocityAgitatorSubsystem::calibrateHere() {

    // drivers->isEmergencyStopActive() is not showing up.

    if(isOnline()){
        agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
        calibrated = true;
        return true;
    }else{
        return false;
    }
}

void VelocityAgitatorSubsystem::clearJam() {
    // No jam functionality for this subsystem

    
}

bool VelocityAgitatorSubsystem::isOnline() {
    return m_agitator.isMotorOnline();
}

float VelocityAgitatorSubsystem::getCurrentValueIntegral() const {
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
}

float VelocityAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    return (2.0f * M_PI / static_cast<float>(tap::motor::DjiMotorEncoder::ENC_RESOLUTION)) *
           m_agitator.getEncoder()->getPosition().getUnwrappedValue()/ AGITATOR_GEAR_RATIO_M2006;
}
}  // namespace control::agitator
