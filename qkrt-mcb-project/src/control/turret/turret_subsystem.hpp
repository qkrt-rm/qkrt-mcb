#pragma once

#include "drivers.hpp"

#include <tap/control/subsystem.hpp>
#include <tap/motor/dji_motor.hpp>
#include <tap/util_macros.hpp>

#include <array>

#include "math/vector.hpp"
#include "math/filter/pid.hpp"

namespace control::turret
{

struct TurretConfig
{
    tap::motor::MotorId pitchId;
    tap::motor::MotorId yawId;
    bool pitchInverted;
    bool yawInverted;
    tap::can::CanBus canBus;
    uint16_t yawForwardOffset;
    uint16_t pitchHorizontalOffset;
};

class TurretSubsystem : public tap::control::Subsystem
{
private:
    using Motor = tap::motor::DjiMotor;
    using Pid = qkrt::Pid<float>;

    static constexpr float MAX_TURRET_MOTOR_RPM = 300.0f;
    static constexpr float MAX_TURRET_MOTOR_VOLTAGE = 25000.0f;
    
    static constexpr float DEAD_ZONE_ANGLE = 0.01f;
    static constexpr float DEAD_ZONE_RPM = 5.0f;
    
    static constexpr float TURRET_MOTOR_GEAR_RATIO = 1.0f;
    static constexpr float INV_ENC_RESOLUTION = 1.0f / static_cast<float>(Motor::ENC_RESOLUTION);
    
    static constexpr float MAX_TURRET_ELEVATION = M_PI_4;

public:
    TurretSubsystem(Drivers& drivers, const TurretConfig& config);

    void initialize() override;
    void refresh() override;
    const char* getName() override { return "Turret"; }

public:
    /**
     * @brief Adjusts the pitch motor to a desired elevation angle
     */
    inline void setElevation(float desiredElevation)
    {
        _M_desiredElevation = std::clamp(desiredElevation, -MAX_TURRET_ELEVATION, MAX_TURRET_ELEVATION);
    }
    
    /**
     * @brief Gets the turret's pitch angle in radians relative to horizontal plane.
     * 
     * @return The pitch angle of the turret (elevation).
     */
    inline float getElevation() const
    {
        return static_cast<float>(_M_pitchHorizontalOffset - _M_pitchMotor.getEncoderWrapped())
             * INV_ENC_RESOLUTION * M_TWOPI;
    }

    /**
     * @brief Adjusts the yaw motor to a desired azimuth angle
     */
    inline void setAzimuth(float desiredAzimuth)
    {   
        _M_desiredAzimuth = desiredAzimuth;
    }

    /**
     * @brief Gets the turret's yaw angle in radians relative to its forward direction.
     * 
     * @return The yaw angle of the turret (azimuth).
     */
    inline float getAzimuth() const
    {
        return static_cast<float>(_M_yawForwardOffset - _M_yawMotor.getEncoderWrapped())
             * INV_ENC_RESOLUTION * M_TWOPI;
    }

    /**
     * @brief Rotates the pitch motor at a specified revolutions per second.
     * 
     * @param pitchRps Desired revolutions per second of the pitch motor.
     */
    void setPitchRps(float pitchRps);

    /**
     * @brief Rotates the yaw motor at a specified revolutions per second.
     * 
     * @param yawRps Desired revolutions per second of the yaw motor.
     */
    void setYawRps(float yawRps);

    void lock() { _M_aimLock = true; }

    void unlock() { _M_aimLock = false; }

private:
    static constexpr float SEC_PER_MIN = 60.0f;

    inline float radPerSecToRpm(float radPerSec) const
    {
        return radPerSec * SEC_PER_MIN / M_TWOPI * TURRET_MOTOR_GEAR_RATIO;
    }

    inline float rpsToRpm(float rps) const
    {
        return rps * SEC_PER_MIN * TURRET_MOTOR_GEAR_RATIO;
    }

    inline float rpsToRadPerSec(float rps) const
    {
        return rps * M_TWOPI;
    }

    Motor _M_pitchMotor, _M_yawMotor;
    float _M_desiredPitchVoltage, _M_desiredYawVoltage;

    float _M_desiredElevation, _M_desiredAzimuth;
    Pid _M_elevationPid, _M_azimuthPid;

    float _M_desiredPitchRpm, _M_desiredYawRpm;
    Pid _M_pitchRpmPid, _M_yawRpmPid;

    bool _M_aimLock;
    float _M_sensitivity;
    uint16_t _M_yawForwardOffset;
    uint16_t _M_pitchHorizontalOffset;
};

};  // namespace control::turret