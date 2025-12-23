/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "edu_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/util_macros.hpp"

using tap::algorithms::limitVal;

namespace control::algorithms
{
float EduPid::runControllerDerivateError(float error, float dt)
{
    // Remove these during implementation
    if (dt == 0) return 0;
    currErrorP = pidConfig.kp * error;
    currErrorI = limitVal<float>(currErrorI + error * dt * pidConfig.ki, -pidConfig.maxICumulative, pidConfig.maxICumulative);
    currErrorD = pidConfig.kd *  (error - prevError) / dt;
    output = limitVal<float>(currErrorP + currErrorI + currErrorD, -pidConfig.maxOutput, pidConfig.maxOutput);
    // STEP 1: implement PID controller
    prevError = error;

    return output;
}

void EduPid::reset()
{
    currErrorP = 0.0f;
    currErrorI = 0.0f;
    currErrorD = 0.0f;
    output = 0.0f;
    prevError = 0.0f;
}
}  // namespace control::algorithms
