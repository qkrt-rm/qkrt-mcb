#include "turret_command.hpp"
#include "turret_kalman_constants.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers & drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& m_operatorInterface)
    : m_drivers(drivers),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      isAutoAim(true),
      m_globalPitch(0.0f), m_globalYaw(0.0f),
      m_pitchCommand(0.0f), m_yawCommand(0.0f),
      m_pitchSensitivity(1.0f), m_yawSensitivity(1.0f),
      m_globalYawTarget(0.0f), m_globalPitchTarget(0.0f),
      m_lastTarget{NAN, NAN, NAN},
      m_kalmanFilter(kalman_config::KALMAN_A, 
                     kalman_config::KALMAN_C, 
                     kalman_config::KALMAN_Q, 
                     kalman_config::KALMAN_R, 
                     kalman_config::KALMAN_P0), 
      m_kalInit(false)
{
    addSubsystemRequirement(&turret);
}

void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    volatile communication::TurretData currentTarget = m_visionCoprocessor.getTurretData();

    bool isNewData = (currentTarget.xPos != m_lastTarget.xPos 
                        || currentTarget.yPos != m_lastTarget.yPos
                        || currentTarget.zPos != m_lastTarget.zPos) && currentTarget.xPos != 0;

    if (m_operatorInterface.isAutoAim())
    {
        m_drivers.digital.set(tap::gpio::Digital::OutputPin::Laser, true);
        static uint32_t loopsWithoutData = 0;

        // No Pitch in Kalman Filter and Ballistics cuz I couldn't get it working

        if(isNewData)
        {
            loopsWithoutData = 0;
            m_turret.lock();

            float cvX = currentTarget.xPos;
            float cvY = currentTarget.yPos - 0.095f; 
            float cvZ = currentTarget.zPos; 

            float flatDist = std::hypot(cvX, cvY);

            // Pitch not used in Kalman Filter and Ballistics
            float aimYawRelative = std::atan2(cvY * -1, currentTarget.xPos);
            float aimPitchRelative = std::atan2(cvZ, flatDist);
                
            m_globalPitchTarget = aimPitchRelative; 

            // Yaw only
            float relYaw = std::atan2(-cvY, cvX);  
            float mathTurretYaw = m_turret.getYaw();      
            float absYaw = mathTurretYaw + relYaw; 

            // world x, y, z *not relative to turret*
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

            // Gets current velocity from Kalman Filter
            m_currentFilteredVel.set(state[3], state[4], state[5]);
        }
        else if (m_kalInit)
        {
            loopsWithoutData++;

            m_currentRawPos.x += m_currentFilteredVel.x * turret::TurretSubsystem::DT;
            m_currentRawPos.y += m_currentFilteredVel.y * turret::TurretSubsystem::DT;

            if (loopsWithoutData > 15) // 30ms without data, lost target
            {
                m_kalInit = false; 
            }
        }

        if (m_kalInit)
        {
            //Balistics
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

            // predicts future target position
            bool solutionFound = tap::algorithms::ballistics::findTargetProjectileIntersection( 
                targetState,
                bulletVelocity,
                iterations,
                &aimPitchAbsolute,
                &aimYawAbsolute,
                &projectedTravelTime,
                m_turret.getPitchOffset()
            );

            if (solutionFound)
            {
                m_globalYawTarget = aimYawAbsolute; 
                
                m_lastTarget.xPos = currentTarget.xPos;
                m_lastTarget.yPos = currentTarget.yPos;
                m_lastTarget.zPos = currentTarget.zPos;  
            }
        }

        // m_logger.printf("Target: %.2f | Actual: %.2f\n", static_cast<double>(m_globalYawTarget), static_cast<double>(m_turret.getYaw()));
        // m_logger.delay(50);

        m_turret.setYaw(m_globalYawTarget);
        m_turret.setPitch(m_globalPitchTarget);
    }
    else
    {
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

        m_turret.ChassisRot(m_operatorInterface.isChassisBeyblade());
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret