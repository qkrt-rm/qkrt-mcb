#include "turret_subsystem.hpp"

using ImuState = tap::communication::sensors::imu::ImuInterface::ImuState;

namespace control::turret
{

TurretSubsystem::TurretSubsystem(Drivers& drivers, const TurretConfig& config)
    : tap::control::Subsystem(&drivers),
      m_pitchMotor(&drivers, config.pitchId, config.canBus, config.pitchInverted, "PITCH"),
      m_yawMotor  (&drivers, config.yawId,   config.canBus, config.yawInverted,   "YAW"),
      m_desiredPitchVoltage(0.0f), m_desiredYawVoltage(0.0f),
      m_desiredPitch(0.0f), m_desiredYaw(0.0f), m_yawPos(0.0f),
      m_pitchPosPid({
          .kp = config.pitchPosGains.kp,
          .ki = config.pitchPosGains.ki,
          .kd = config.pitchPosGains.kd,
          .maxICumulative = config.pitchPosGains.maxICumulative,
          .maxOutput = config.pitchPosGains.maxOutput,
      }),
      m_yawPosPid({
          .kp = config.yawPosGains.kp,
          .ki = config.yawPosGains.ki,
          .kd = config.yawPosGains.kd,
          .maxICumulative = config.yawPosGains.maxICumulative,
          .maxOutput = config.yawPosGains.maxOutput,
      }),
      m_desiredPitchRps(0.0f), m_desiredYawRps(0.0f),
      m_pitchVelPid({
          .kp = config.pitchVelGains.kp,
          .ki = config.pitchVelGains.ki,
          .kd = config.pitchVelGains.kd,
          .maxICumulative = config.pitchVelGains.maxICumulative,
          .maxOutput = config.pitchVelGains.maxOutput,
      }),
      m_yawVelPid({
          .kp = config.yawVelGains.kp,
          .ki = config.yawVelGains.ki,
          .kd = config.yawVelGains.kd,
          .maxICumulative = config.yawVelGains.maxICumulative,
          .maxOutput = config.yawVelGains.maxOutput,
      }),

      m_isCalibrated(false),
      m_aimLock(false),  
      m_isChassisRot(false),
      m_mcbHoriz(config.mcbHoriz),
      m_sensitivity(1.0f),
      m_yawOffset(config.yawForwardOffset),
      m_pitchOffset(config.pitchHorizontalOffset),
      m_pitchUpLim(config.pitchUpLim),
      m_pitchDownLim(config.pitchDownLim),
      m_maxPitchPower(config.MAX_PITCH_POWER),
      m_maxYawPower(config.MAX_YAW_POWER),
      m_maxRps(config.MAX_RPS),
      m_imu(drivers.bmi088),
      m_drivers(&drivers),
      m_logger(drivers.logger)
{   
}

void TurretSubsystem::initialize()
{
    m_pitchMotor.initialize();
    m_yawMotor.initialize();

    m_pitchPosPid.reset();
    m_yawPosPid.reset();
    m_pitchVelPid.reset();
    m_yawVelPid.reset();
}

void TurretSubsystem::refresh()
{
    // REMOVED: m_yawPos calibration tracking block from here.

    if(m_drivers->isEmergencyStopActive() || m_imu.getImuState() == ImuState::IMU_CALIBRATING) 
    {
        m_pitchPosPid.reset();
        m_yawPosPid.reset();
        m_yawVelPid.reset();
        m_pitchVelPid.reset();

        m_desiredPitchVoltage = 0.0f;
        m_desiredYawVoltage = 0.0f;
    }
    else if (m_aimLock)     
    {
        float currPitch = getPitch();

        // Feed forward logic using cos()
        float pitchKFF = 2600.0f;
        float pitchFF = pitchKFF * std::cos(currPitch);

        // Pitch Loop
        float pitchError = getOptimalError(m_desiredPitch, currPitch);
        float desiredPitchRpm = m_pitchPosPid.runControllerDerivateError(pitchError, DT);

        float currentPitchRpm = m_pitchMotor.getEncoder()->getVelocity();  
        m_pitchVelPid.runControllerDerivateError(desiredPitchRpm - currentPitchRpm, DT);
        
        m_desiredPitchVoltage = m_pitchVelPid.getOutput() + pitchFF;

        // Yaw Loop
        float currentYawRpm = m_imu.getGz() * -1;  
        
        // FIX: Replaced m_yawPos tracking with direct getYaw() and getOptimalError()
        float azimuthError = getOptimalError(m_desiredYaw, getYaw());
        float desiredYawRpm = m_yawPosPid.runController(azimuthError, currentYawRpm, DT);

        m_desiredYawVoltage = m_yawVelPid.runController(desiredYawRpm - currentYawRpm, 0.0f, DT);
    }
    else 
    {
        // Manual Velocity PID
        float pitchKFF = 1000.0f;
        float yawKFF = 6560.0f;
    
        float currPitch = getPitch();

        float yawFF = (m_isChassisRot) ? (yawKFF * -chassis::HolonomicChassisCommand::CHASSIS_ROT_SPEED_RAD) : 0.0f;
        
        float pitchFF = pitchKFF * std::cos(currPitch);

        // Pitch Loop
        float pitchError = m_desiredPitch - currPitch;
        float desiredPitchRps = m_pitchPosPid.runControllerDerivateError(pitchError, DT);

        float pitchRpsError = desiredPitchRps - (m_pitchMotor.getEncoder()->getVelocity());
        m_pitchVelPid.runControllerDerivateError(pitchRpsError, DT);
        m_desiredPitchVoltage = m_pitchVelPid.getOutput() + pitchFF;

        // Yaw Loop 
        float imuYawRps = (m_mcbHoriz ? m_imu.getGz() * -1 : m_imu.getGx());

        float yawRpsError = m_desiredYawRps - imuYawRps;
        m_yawVelPid.runControllerDerivateError(yawRpsError, DT);
        m_desiredYawVoltage = m_yawVelPid.getOutput() + yawFF;
    }

    m_desiredPitchVoltage = std::clamp(m_desiredPitchVoltage, -m_maxPitchPower, m_maxPitchPower);
    m_desiredYawVoltage = std::clamp(m_desiredYawVoltage, -m_maxYawPower, m_maxYawPower);

    m_pitchMotor.setDesiredOutput(m_desiredPitchVoltage);
    m_yawMotor.setDesiredOutput(m_desiredYawVoltage);
}

void TurretSubsystem::setPitchRps(float pitchRps)
{
    {
        m_desiredPitchRps = std::clamp(
            pitchRps * (2.0f * static_cast<float>(M_PI)) * m_sensitivity, 
            -m_maxRps, m_maxRps
        );
    }
}

void TurretSubsystem::setYawRps(float yawRps)
{
    m_desiredYawRps = std::clamp(
        yawRps * (2.0f * static_cast<float>(M_PI)) * m_sensitivity, 
        -m_maxRps, m_maxRps
    );
}

void TurretSubsystem::ChassisRot(bool isRot) { m_isChassisRot = isRot; }

}  // namespace control::turret