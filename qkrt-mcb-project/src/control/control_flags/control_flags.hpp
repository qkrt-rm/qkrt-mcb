#pragma once

namespace control {

/// @brief Namespace for common settings that is shared among the subsystems.
namespace flags {

    // Default PWM value that ensures the motor is turned off
    inline constexpr float OFF_PWM = 0.25f;

}
}