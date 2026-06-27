#include "auto_turret_command.hpp"
#include "turret_kalman_constants.hpp"


namespace control::turret
{

AutoTurretCommand::AutoTurretCommand(Drivers & drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& m_operatorInterface,
                             tap::control::Command* flywheelsCommand, ///
                             tap::control::Command* agitatorCommand) ///
    : m_drivers(drivers),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_flywheelsCommand(flywheelsCommand), ///
      m_agitatorCommand(agitatorCommand), ///
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      isAutoAim(true),
      m_globalPitch(0.0f), m_globalYaw(0.0f),
      m_pitchCommand(0.0f), m_yawCommand(0.0f),
      m_pitchSensitivity(1.0f), m_yawSensitivity(1.0f),
      m_globalYawTarget(0.0f), m_globalPitchTarget(0.0f),
      m_pitchBoresightTrim(0.0f),
      m_yawBoresightTrim(0.05f),
      m_lastTarget{NAN, NAN, NAN},
      m_currentState(SentryState::SCANNING), 
      m_targetLostTicks(0),
      m_targetStartTicks(0),
      m_targetAcquireTicks(0),
      m_scanDirection(1.0f),
      m_kalmanFilter(kalman_config::KALMAN_A, 
                     kalman_config::KALMAN_C, 
                     kalman_config::KALMAN_Q, 
                     kalman_config::KALMAN_R, 
                     kalman_config::KALMAN_P0), 
      m_kalInit(false),
      m_pitchFilter(0.05f, 1.0f)
{
    addSubsystemRequirement(&turret);
}

void AutoTurretCommand::initialize()
{
}

void AutoTurretCommand::execute()
{
    volatile communication::TurretData currentTarget = m_visionCoprocessor.getTurretData();

    bool isNewData = (currentTarget.xPos != m_lastTarget.xPos 
                        || currentTarget.yPos != m_lastTarget.yPos
                        || currentTarget.zPos != m_lastTarget.zPos) && currentTarget.xPos != 0;

    // -----------------------------------------
    // Phase 1: State Transitions
    // -----------------------------------------
    if (m_operatorInterface.isAutoAim())
    {
        
        switch (m_currentState)
        {
            case SentryState::SCANNING:
                if (isNewData)
                {
                    m_targetAcquireTicks++;
                    if (m_targetAcquireTicks >= TARGET_ACQUIRE_TICKS)
                    {
                        m_currentState = SentryState::SHOOTING;
                        m_targetLostTicks = 0;
                        m_targetStartTicks = 0;
                        m_targetAcquireTicks = 0;
                        m_pitchFilter.reset();
                    }
                }
                else
                {
                    m_targetAcquireTicks = 0;
                }
                break;

            case SentryState::SHOOTING:
                m_targetStartTicks++;


                if (!isNewData)
                {
                    m_targetLostTicks++;
                    if (m_targetLostTicks >= TARGET_LOST_TIMEOUT_TICKS)
                    {
                        m_currentState = SentryState::SCANNING;
                        m_kalInit = false; // Reset Kalman filter on target drop
                    }
                }
                else
                {
                    m_targetLostTicks = 0;
                }
                break;
        }
    }
    else
    {
        m_currentState = SentryState::SCANNING; 
    }

    // -----------------------------------------
    // Phase 2: State Behaviors
    // -----------------------------------------
    if (m_operatorInterface.isAutoAim())
    {
        if (m_currentState == SentryState::SHOOTING)
        {
            if (m_targetStartTicks >= TARGET_START_SHOOTING_TICKS)
            {
                m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, true); ///
                m_drivers.commandScheduler.addCommand(m_agitatorCommand); ///
            }
            m_turret.lock();

            static uint32_t loopsWithoutData = 0;

            if(isNewData)
            {
                loopsWithoutData = 0;

                float cvX = currentTarget.xPos;
                float cvY = currentTarget.yPos - 0.045f; 
                float cvZ = currentTarget.zPos + 0.75f; 

                float flatDist = std::hypot(cvX, cvY);

                // FIX: Assign direct relative aim, NO compounded turret angle
                float aimPitchRelative = std::atan2(cvZ, flatDist) + m_pitchBoresightTrim;
                float aimPitchAbsoluteMeasured = m_turret.getPitch() + aimPitchRelative;
                float filteredPitch = m_pitchFilter.filterData(aimPitchAbsoluteMeasured);

                m_globalPitchTarget = std::clamp(
                    filteredPitch,
                    m_globalPitchTarget - MAX_PITCH_STEP_PER_UPDATE,
                    m_globalPitchTarget + MAX_PITCH_STEP_PER_UPDATE
                );

                // Yaw relative calculation
                float relYaw = std::atan2(-cvY, cvX) + m_yawBoresightTrim; 
                float mathTurretYaw = m_turret.getYaw();      
                float absYaw = mathTurretYaw + relYaw; 

                // World coordinates
                float worldX = flatDist * std::cos(absYaw);
                float worldY = flatDist * std::sin(absYaw);
                float worldZ = 0.0f; 

                m_currentRawPos.set(worldX, worldY, worldZ);

                // Kalman Filter
                tap::algorithms::CMSISMat<K_INPUTS, 1> measurement;
                measurement.data[0] = worldX;
                measurement.data[1] = worldY;
                measurement.data[2] = worldZ;

                if (!m_kalInit)
                {
                    float initialState[K_STATES] = {worldX, worldY, worldZ, 0.0f, 0.0f, 0.0f};
                    m_kalmanFilter.init(initialState);
                    m_kalInit = true;
                }
                else
                {
                    m_kalmanFilter.performUpdate(measurement);
                }

                const auto& state = m_kalmanFilter.getStateVectorAsMatrix();
                m_currentFilteredVel.set(state[3], state[4], state[5]);
            }
            else if (m_kalInit)
            {
                loopsWithoutData++;

                m_currentRawPos.x += m_currentFilteredVel.x * turret::TurretSubsystem::DT;
                m_currentRawPos.y += m_currentFilteredVel.y * turret::TurretSubsystem::DT;

                if (loopsWithoutData > 15)
                {
                    m_kalInit = false; 
                }
            }

            if (m_kalInit)
            {
                // Ballistics
                modm::Vector3f zeroAccel(0.0f, 0.0f, 0.0f);
                modm::Vector3f stableVel(m_currentFilteredVel.x, -m_currentFilteredVel.y, 0.0f);

                tap::algorithms::ballistics::SecondOrderKinematicState targetState(
                    m_currentRawPos,
                    stableVel,
                    zeroAccel
                );

                float bulletVelocity = 15.0f; 
                uint8_t iterations = 3;
                float projectedTravelTime = 0.0f;
                float aimPitchAbsolute = 0.0f;
                float aimYawAbsolute = 0.0f;

                bool solutionFound = tap::algorithms::ballistics::findTargetProjectileIntersection( 
                    targetState,
                    bulletVelocity,
                    iterations,
                    &aimPitchAbsolute,
                    &aimYawAbsolute,
                    &projectedTravelTime,
                    m_turret.getPitchAxisOffsetMeters()
                );

                if (solutionFound)
                {
                    m_globalYawTarget = aimYawAbsolute;
                    m_lastTarget.xPos = currentTarget.xPos;
                    m_lastTarget.yPos = currentTarget.yPos;
                    m_lastTarget.zPos = currentTarget.zPos;  
                }
            }

            m_turret.setYaw(m_globalYawTarget);
            m_turret.setPitch(m_globalPitchTarget);
        }
        else if (m_currentState == SentryState::SCANNING)
        {
            m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, false);
            m_turret.unlock();

            // Automatic sweeping logic
            const float SCAN_SPEED_RPS = 0.2f; 
            
            // Re-implement your bounds check here if you need it to bounce back and forth
            m_scanDirection = -1.0f;

            m_turret.setYawRps(SCAN_SPEED_RPS * m_scanDirection);
            m_turret.setPitch(-0.17f);

            m_drivers.commandScheduler.addCommand(m_flywheelsCommand); ///
            m_drivers.commandScheduler.removeCommand(m_agitatorCommand, true);
        }
    }
    else
    {
        m_logger.printf("Current State: %f \n", m_turret.getPitch());
        m_logger.delay(400);
        m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, false);

        //Manual Velocity Control 
        m_operatorInterface.pollInputDevices();

        m_turret.unlock(); 

        float pitchInp = m_operatorInterface.getTurretPitchInput();
        float yawInp = m_operatorInterface.getTurretYawInput();
        
        m_pitchCommand = pitchInp * (2.0f * static_cast<float>(M_PI)) * m_pitchSensitivity * (turret::TurretSubsystem::DT);
        m_globalPitch += m_pitchCommand;

        m_globalPitch = std::clamp(m_globalPitch, m_turret.m_pitchDownLim, m_turret.m_pitchUpLim);

        m_turret.setPitch(m_globalPitch);
        m_turret.setYawRps(yawInp);

        m_turret.isChassisRot(m_operatorInterface.getChassisBeyblade());

        m_currentState = SentryState::SCANNING;

        m_drivers.commandScheduler.removeCommand(m_flywheelsCommand, true); ///
        m_drivers.commandScheduler.removeCommand(m_agitatorCommand, true);
    }
}

void AutoTurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret