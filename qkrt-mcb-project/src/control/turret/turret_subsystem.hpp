#pragma once

#include "drivers.hpp"

#include <tap/control/subsystem.hpp>
#include <tap/motor/dji_motor.hpp>
#include <tap/util_macros.hpp>
#include <tap/algorithms/extended_kalman.hpp>
#include <tap/algorithms/filter/butterworth.hpp>
#include <tap/algorithms/filter/discrete_filter.hpp>
#include "communication/logger/logger.hpp"


#include <array>

#include <numbers>
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
    static constexpr float DEAD_ZONE_RPM = 0.5f;
        
    static constexpr float MAX_TURRET_ELEVATION = M_PI_4;

    static constexpr double LPF_SAMPLE_TIME = 0.002;
    static constexpr double LPF_CUTOFF_HZ = 40.0;

public:
    TurretSubsystem(Drivers& drivers, const TurretConfig& config);

    void initialize() override;
    void refresh() override;
    const char* getName() const override { return "Turret"; }

public:
    /**
     * @brief Adjusts the pitch motor to a desired elevation angle
     */
    inline void setElevation(float desiredElevation)
    {
        m_desiredElevation = std::clamp(desiredElevation, -MAX_TURRET_ELEVATION, MAX_TURRET_ELEVATION);
    }
    
    /**
     * @brief Gets the turret's pitch angle in radians relative to horizontal plane.
     * 
     * @return The pitch angle of the turret (elevation).
     */
    inline float getElevation() const
    {
        auto currentAngle = m_pitchMotor.getEncoder()->getPosition();
        return (currentAngle - m_pitchHorizontalOffset).getWrappedValue();
    }

    /**
     * @brief Adjusts the yaw motor to a desired azimuth angle
     */
    inline void setAzimuth(float desiredAzimuth)
    {   
        m_desiredAzimuth = desiredAzimuth;
    }

    /**
     * @brief Gets the turret's yaw angle in radians relative to its forward direction.
     * 
     * @return The yaw angle of the turret (azimuth).
     */
    inline float getAzimuth() const
    {
        auto currentAngle = m_yawMotor.getEncoder()->getPosition();
        return (currentAngle - m_yawForwardOffset).getWrappedValue();
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

    void lock() { m_aimLock = true; }

    void unlock() { m_aimLock = false; }

private:
    inline float encoderToRad(uint16_t encoder) const
    {
        static constexpr float INV_ENC_RESOLUTION
            = 1.0f / static_cast<float>(tap::motor::DjiMotorEncoder::ENC_RESOLUTION);

        return static_cast<float>(encoder) * INV_ENC_RESOLUTION * M_TWOPI;
    }

    inline float rpsToRpm(float rps) const
    {
        static constexpr float SEC_PER_MIN = 60.0f;
        static constexpr float TURRET_MOTOR_GEAR_RATIO = 1.0f;

        return rps * SEC_PER_MIN * TURRET_MOTOR_GEAR_RATIO;
    }

    Motor m_pitchMotor, m_yawMotor;
    float m_desiredPitchVoltage, m_desiredYawVoltage;

    float m_desiredElevation, m_desiredAzimuth;
    Pid m_elevationPid, m_azimuthPid;

    float m_desiredPitchRpm, m_desiredYawRpm;
    Pid m_pitchRpmPid, m_yawRpmPid;

    bool m_aimLock;
    float m_sensitivity;
    float m_yawForwardOffset;
    float m_pitchHorizontalOffset;

    tap::algorithms::ExtendedKalman m_ImuKalman;
    tap::algorithms::filter::DiscreteFilter<3, float> m_ImuLpf;

    tap::communication::sensors::imu::bmi088::Bmi088& m_imu;
    Drivers* m_drivers;

    communication::logger::Logger& m_logger;

};

};  // namespace control::turret