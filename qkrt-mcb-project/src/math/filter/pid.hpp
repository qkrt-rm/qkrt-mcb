#pragma once

#include <algorithm>
#include <limits>

namespace qkrt
{

template <typename T>
class Pid
{
public:
    Pid(const T& kp, const T& ki, const T& kd,
        const T& maxOutput = 0, const T& maxErrorSum = 0)
        : kp(kp), ki(ki), kd(kd),
          prevError(0), errorSum(0),
          maxOutput(maxOutput), maxErrorSum(maxErrorSum),
          output(0)
    {
        // initialize `maxOutput` to "unlimited" if not provided
        this->maxOutput = (maxOutput == 0)
                        ? std::numeric_limits<T>::max()
                        : maxOutput;

        // calculate `maxErrorSum` if not provided
        this->maxErrorSum = (maxErrorSum == 0 && this->maxOutput > 0 && ki > 0)
                          ? (this->maxOutput / ki)
                          : maxErrorSum;
    }

    void update(const T& error)
    {
        errorSum += error;

        T integral = std::clamp(errorSum, -maxErrorSum, maxErrorSum);
        T derivative = error - prevError;

        output = kp * error
               + ki * integral
               + kd * derivative;
        output = std::clamp(output, -maxOutput, maxOutput);

        prevError = error;
        errorSum = integral;
    }

    T getValue() const { return output; }

private:
    T kp, ki, kd;
    T prevError, errorSum;
    T maxOutput, maxErrorSum;
    T output;
};

}  // namespace qkrt