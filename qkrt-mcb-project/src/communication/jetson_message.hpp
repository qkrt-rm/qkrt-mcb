#pragma once
#include <stdint.h>

namespace communication
{
    enum MessageTypes : uint8_t
    {
        JETSON_MESSAGE_TYPE_AIM = 1,
        JETSON_MESSAGE_TYPE_NAV = 2,
        MCB_MESSAGE_TYPE_ODOM = 3,
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

    //TODO: adjust for what auto want
    struct ImuData
    {
        float xAcl;
        float yAcl;
        float zGyro;
    } modm_packed;

}