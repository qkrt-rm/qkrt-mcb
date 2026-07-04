#include "auto_holonomic_chassis_command.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::chassis
{

AutoHolonomicChassisCommand::AutoHolonomicChassisCommand(Drivers &drivers, HolonomicChassisSubsystem& chassis,
                                                 turret::TurretSubsystem& turret,
                                                 ControlOperatorInterface& m_operatorInterface,
                                                 chassisCommandConfig config)
    : m_chassis(chassis),
      m_turret(turret),
      m_operatorInterface(m_operatorInterface),
      m_maxSpeed(config.maxChassisSpeed),
      m_chassisRotSpeed(config.maxRotSpeed),
      m_startTimer(0.0f),
      isNavReady(false),
      m_sequenceTimer(0.0f),
      m_visionCoprocessor(drivers.visionCoprocessor),
      m_logger(drivers.logger),
      m_drivers(&drivers),
      isHardCode(true),
      islockTurret(false)

{
    addSubsystemRequirement(&chassis);
}

void AutoHolonomicChassisCommand::initialize()
{
}

void AutoHolonomicChassisCommand::execute()
{       
    m_operatorInterface.pollInputDevices();

    // ==========================================
    // 1. READ SENSORS, REFEREE, & STATE
    // ==========================================
    m_globalYaw = m_drivers->bmi088.getYaw(); 
    communication::AprilTagData tagData = m_visionCoprocessor.getAprilTagData();
    
    auto robotData = m_drivers->refSerial.getRobotData();
    uint16_t currentHealth = robotData.currentHp;
    bool isRedTeam = (robotData.robotId == tap::communication::serial::RefSerialData::RobotId::RED_SENTINEL);

    // ==========================================
    // THE INSTANT-START OVERRIDE
    // ==========================================
    // True = Robot takes over immediately. False = Manual RC control.
    bool forceAutoNav = true; 
    
    float rawInpX = 0.0f;  
    float rawInpY = 0.0f;  
    float w = 0.0f;        
    float yawAngle = 0.0f; 

    // ==========================================
    // 2. SENSOR FUSION: THE ANCHOR RESET
    // ==========================================
    if (tagData.tagId != 0 && tagData.zDist > 0.0f) 
    {
        TagLocation anchor = getTagLocation(tagData.tagId);
        float global_x_offset = (tagData.zDist * std::cos(m_globalYaw)) - (-tagData.xDist * std::sin(m_globalYaw));
        float global_y_offset = (tagData.zDist * std::sin(m_globalYaw)) + (-tagData.xDist * std::cos(m_globalYaw));

        m_globalX = anchor.x - global_x_offset;
        m_globalY = anchor.y - global_y_offset;
    } 

    // ==========================================
    // 3. THE HIGH-LEVEL BEHAVIOR STATE MACHINE
    // ==========================================
    if (forceAutoNav)
    {
        // Determine if we are in the center zone based on team
        bool inCenterZone = isRedTeam ? (m_globalX >= 4.5f) : (m_globalX <= 7.5f);
        
        enum class SentryState { RETREAT, RUSH_MID, DEFEND_MID };
        SentryState currentState;

        // Condition Hierarchy (Checking > 0 prevents the pit-testing 0 HP trap!)
        if (currentHealth < 100 && currentHealth > 0) {
            currentState = SentryState::RETREAT;
        } else if (!inCenterZone) {
            currentState = SentryState::RUSH_MID;
        } else {
            currentState = SentryState::DEFEND_MID;
        }

        // --- WAYPOINT SELECTOR ---
        float targetX = 0.0f; 
        float targetY = 0.0f;

        if (currentState == SentryState::DEFEND_MID)
        {
            // COMBAT/PATROL MODE
            yawAngle = 0.0f;      // Decouple chassis
            islockTurret = false; // Turret tracks enemies freely

            // Alternate targetY every 2 seconds to patrol the line
            m_sequenceTimer += 0.002f;
            float combatTimer = std::fmod(m_sequenceTimer, 4.0f); 
            
            targetX = isRedTeam ? 5.0f : 7.0f;
            targetY = (combatTimer < 2.0f) ? 5.0f : 3.0f; 
        }
        else 
        {
            // TRANSIT MODE (Rush or Retreat)
            islockTurret = true;          // Lock turret forward
            yawAngle = m_turret.getYaw(); // Couple chassis to turret

            if (isRedTeam) 
            {
                if (currentState == SentryState::RUSH_MID) {
                    if (m_globalX < 2.0f)      { targetX = 1.0f; targetY = 1.25f; } // WP1
                    else if (m_globalX < 4.5f) { targetX = 3.5f; targetY = 1.50f; } // WP2
                    else                       { targetX = 5.0f; targetY = 4.00f; } // Push Mid
                } else { // RETREAT
                    if (m_globalX > 4.5f)      { targetX = 3.5f; targetY = 1.50f; } // WP2
                    else if (m_globalX > 2.0f) { targetX = 1.0f; targetY = 1.25f; } // WP1
                    else                       { targetX = 0.5f; targetY = 4.00f; } // Spawn
                }
            } 
            else // BLUE TEAM
            {
                if (currentState == SentryState::RUSH_MID) {
                    if (m_globalX > 10.0f)     { targetX = 11.0f; targetY = 1.25f; } // WP1
                    else if (m_globalX > 7.5f) { targetX = 8.5f;  targetY = 1.50f; } // WP2
                    else                       { targetX = 7.0f;  targetY = 4.00f; } // Push Mid
                } else { // RETREAT
                    if (m_globalX < 7.5f)      { targetX = 8.5f;  targetY = 1.50f; } // WP2
                    else if (m_globalX < 10.0f){ targetX = 11.0f; targetY = 1.25f; } // WP1
                    else                       { targetX = 11.5f; targetY = 4.00f; } // Spawn
                }
            }
        }

        // --- GLOBAL P-CONTROLLER ---
        // We use this for BOTH Transit and Defend!
        float errorX = targetX - m_globalX;
        float errorY = targetY - m_globalY;

        float local_x_cmd = (errorX * std::cos(-m_globalYaw)) - (errorY * std::sin(-m_globalYaw));
        float local_y_cmd = (errorX * std::sin(-m_globalYaw)) + (errorY * std::cos(-m_globalYaw));

        // Adjust Kp based on testing. 1.5 is a solid starting point.
        rawInpX = local_x_cmd * 1.5f; 
        rawInpY = local_y_cmd * 1.5f; 
    }
    else
    {
        // Fallback to manual operator control
        rawInpX = m_operatorInterface.getChassisXInput();
        rawInpY = m_operatorInterface.getChassisYInput();
        yawAngle = m_turret.getYaw();
    }

    // ==========================================
    // 4. KINEMATICS & NORMALIZATION
    // ==========================================
    Vector2f moveVector(rawInpX, rawInpY);
    float inputLength = moveVector.getLength();
    if (inputLength > 1.0f) moveVector = moveVector / inputLength;

    Vector2f scaledMove = moveVector * m_maxSpeed;
    float xInp = scaledMove.x;
    float yInp = scaledMove.y;
    
    float v_y = yInp * std::cos(-yawAngle) - xInp * std::sin(-yawAngle);
    float v_x = yInp * std::sin(-yawAngle) + xInp * std::cos(-yawAngle);
    
    float leftFront  = (v_x + v_y + w);
    float leftBack   = (v_x - v_y + w);
    float rightFront = (v_x - v_y - w);
    float rightBack  = (v_x + v_y - w);

    m_chassis.setWheelVelocities(leftFront, leftBack, rightBack, rightFront);

    // ==========================================
    // 5. DEAD RECKONING (IF BLIND)
    // ==========================================
    if (tagData.tagId == 0 || tagData.zDist <= 0.0f) 
    {
        float dt = 0.002f; 
        float field_vx = (xInp * std::cos(m_globalYaw)) - (yInp * std::sin(m_globalYaw));
        float field_vy = (xInp * std::sin(m_globalYaw)) + (yInp * std::cos(m_globalYaw));

        m_globalX += field_vx * dt;
        m_globalY += field_vy * dt;
    }
}

void AutoHolonomicChassisCommand::end(bool /* interrupted */)
{
    m_chassis.setWheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
}


}  // namespace control::chassis
