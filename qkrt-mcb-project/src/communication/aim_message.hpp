#pragma once
#include <stdint.h>
#include <chrono>

namespace communication
{
    struct TurretData
    {
        float xPos;
        float yPos;
        float zPos;
        // std::chrono::steady_clock::time_point timestamp;
    } modm_packed;
}