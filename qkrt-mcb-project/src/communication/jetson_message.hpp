#pragma once
#include <stdint.h>

namespace communication
{
    enum MessageTypes : uint8_t
    {
        JETSON_MESSAGE_TYPE_AIM = 1,
        JETSON_MESSAGE_TYPE_NAV = 2,
    };

    struct TurretData
    {
        float xPos;
        float yPos;
        float zPos;
    } modm_packed;

    struct NavData
    {
        float xVel;
        float yVel;
        float wVel;
    } modm_packed;

}