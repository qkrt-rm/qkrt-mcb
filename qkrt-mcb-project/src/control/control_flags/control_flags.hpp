#pragma once

namespace control {

/// @brief Namespace for common settings that is shared among the subsystems.
namespace flags {
    //extern bool emergency_stop = false;

    // Default PWM value that ensures the motor is turned off
    extern constexpr float OFF_PWM = 0.25f;
}
}