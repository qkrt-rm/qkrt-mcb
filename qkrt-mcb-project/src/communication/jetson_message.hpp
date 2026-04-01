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

    struct OdomData
    {
        // Chassis wheel encoder velocities (rad/s)
        float wheelLF;
        float wheelLB;
        float wheelRB;
        float wheelRF;
        // Turret encoder positions (rad) and velocities (rad/s)
        float turretYawPos;
        float turretYawVel;
        float turretPitchPos;
        float turretPitchVel;
        // IMU accelerometer (m/s²)
        float imuAx;
        float imuAy;
        float imuAz;
        // IMU gyroscope (rad/s)
        float imuGx;
        float imuGy;
        float imuGz;
        // IMU Mahony orientation (rad)
        float imuYaw;
        float imuPitch;
        float imuRoll;
    } modm_packed;

}