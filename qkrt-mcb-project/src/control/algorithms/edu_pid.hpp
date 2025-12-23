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

#pragma once

namespace control::algorithms
{
/// Gains and constants, to be set by the user.
struct EduPidConfig
{
    const float kp{};
    const float ki{};
    const float kd{};
    const float maxICumulative{};
    const float maxOutput{};
};

/**
 * A base PID controller that uses current error to calculate some desired output.
 * To use, declare an EduPid class then in a loop, call the runControllerDerivativeError
 * function over and over, feeding in the current error between the setpoint and actual
 * value and the time since the last call to runControllerDerivativeError.
 */
class EduPid
{
public:
    EduPid(const EduPidConfig &pidConfig) : pidConfig(pidConfig) {}

    /**
     * Updates the PID controller. Takes one step of the following form:
     *
     * \f$ u(t) = K_p e(t) + K_i \sum_{x=0}^t e(x) \Delta_t + K_d \Delta_t (e(t)-e(t-1) \f$
     *
     * Where \f$u(t)\f$ is the output, \f$K_p, K_i, K_d\f$ are the proportionality constants,
     * and \f$\Delta_t\f$ is the time step between the previous and current iteration.
     *
     * @param[in] error the error between the desired and actual value.
     * @param[in] dt the time difference between the previous and current iteration.
     * @see tap::arch::clock for measuring time.
     * @return the new output calculated by the PID controller.
     */
    float runControllerDerivateError(float error, float dt);

    /**
     * @return the last output calculated during `runControllerDerivativeError`.
     */
    float getOutput() const { return output; }

    /**
     * Zeros the p, i, d, and output terms.
     */
    void reset();

private:
    const EduPidConfig &pidConfig;

    // While these could be local, debugging pid is much easier if they are not.
    float currErrorP{0};
    float currErrorI{0};
    float currErrorD{0};
    float output{0};
    float prevError{0};
};
}  // namespace control::algorithms
