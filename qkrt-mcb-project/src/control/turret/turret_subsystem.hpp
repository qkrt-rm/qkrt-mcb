#pragma once

#include "drivers.hpp"

#include <tap/control/subsystem.hpp>
#include <tap/motor/dji_motor.hpp>
#include <tap/util_macros.hpp>
#include <tap/algorithms/extended_kalman.hpp>
#include <tap/algorithms/filter/butterworth.hpp>
#include <tap/algorithms/filter/discrete_filter.hpp>
#include <tap/algorithms/smooth_pid.hpp>
#include "communication/logger/logger.hpp"
#include "control/chassis/holonomic_chassis_command.hpp"

#include <array>

#include <numbers>
#include "math/vector.hpp"
// #include "math/filter/pid.hpp"

namespace control::turret
{

struct PidGains
{
    float kp, ki, kd;
    float maxICumulative;
    float maxOutput;

    float tQ = 1.0f;
    float tR = 0.0f;
};

struct TurretConfig
{
    //hardware settings
    tap::motor::MotorId pitchId;
    tap::motor::MotorId yawId;
    bool pitchInverted;
    bool yawInverted;
    bool mcbHoriz; 
    tap::can::CanBus canBus;

    //offsets and limits
    float yawForwardOffset;
    float pitchHorizontalOffset;
    float pitchUpLim;
    float pitchDownLim;
    float MAX_PITCH_POWER;
    float MAX_YAW_POWER;
    float MAX_RPS;

    //PID gains
    PidGains pitchPosGains;
    PidGains pitchVelGains;
    PidGains yawPosGains;
    PidGains yawVelGains;
};

class TurretSubsystem : public tap::control::Subsystem
{

public:
    static constexpr float DT = 0.002f;
    
    float m_pitchUpLim;
    float m_pitchDownLim;

    TurretSubsystem(Drivers& drivers, const TurretConfig& config);

    void initialize() override;
    void refresh() override;
    const char* getName() const override { return "Turret"; }

    /**
     * @brief Adjusts the pitch motor to a desired elevation angle
     */
    inline void setPitch(float pitchInp)
    {
        m_desiredPitch = std::clamp(pitchInp, m_pitchDownLim, m_pitchUpLim);
    }
    
    /**
     * @brief Gets the turret's pitch angle in radians relative to horizontal plane.
     * 
     * @return The pitch angle of the turret (elevation).
     */
    inline float getPitch() const
    {
        auto relativeAngle = m_pitchMotor.getEncoder()->getPosition() + m_pitchOffset;
        return (relativeAngle).getUnwrappedValue();
    }

    /**
     * @brief Adjusts the yaw motor to a desired azimuth angle
     */
    inline void setYaw(float YawInp)
    {   
        m_desiredYaw = YawInp;
    }

    /**
     * @brief Gets the turret's yaw angle in radians relative to its forward direction.
     * 
     * @return The yaw angle of the turret (azimuth).
     */
    inline float getYaw() const
    {
        auto currentAngle = m_yawMotor.getEncoder()->getPosition() + m_yawOffset;
        return (currentAngle).getWrappedValue();
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

    void ChassisRot(bool isRot);


private:

    using Motor = tap::motor::DjiMotor;

    Motor m_pitchMotor, m_yawMotor;
    float m_desiredPitchVoltage, m_desiredYawVoltage;

    float m_desiredPitch, m_desiredYaw;

    tap::algorithms::SmoothPid m_pitchPosPid;
    tap::algorithms::SmoothPid m_yawPosPid;

    float m_desiredPitchRps, m_desiredYawRps;

    tap::algorithms::SmoothPid m_pitchVelPid;
    tap::algorithms::SmoothPid m_yawVelPid;

    bool m_isCalibrated;
    bool m_aimLock;
    bool m_isChassisRot;
    bool m_mcbHoriz;

    float m_sensitivity;
    
    float m_yawOffset;
    float m_pitchOffset;

    float m_maxPitchPower;
    float m_maxYawPower;
    float m_maxRps;

    tap::communication::sensors::imu::bmi088::Bmi088& m_imu;
    Drivers* m_drivers;

    communication::logger::Logger& m_logger;

        /**
     * @brief Computes the shortest angular error between two angles.
     *
     * This function calculates the smallest difference between a desired angle 
     * and the current angle, ensuring the result is within the range [-π, π]. 
     * This prevents issues where an angle error of, for example, 350° would be 
     * treated as -350° instead of 10°.
     *
     * @param desiredAngle The target angle in radians.
     * @param currentAngle The current angle in radians.
     * @return The shortest angular difference in radians, constrained to [-π, π].
     */
    static constexpr auto getOptimalError = [](float desiredAngle, float currentAngle) -> float
        {
            float error = desiredAngle - currentAngle;
            return std::atan2(std::sin(error), std::cos(error));
        };


};

};  // namespace control::turret