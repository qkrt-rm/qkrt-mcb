#include "holonomic_chassis_subsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace control::chassis
{

HolonomicChassisSubsystem::HolonomicChassisSubsystem(Drivers& drivers, const ChassisConfig& config)
    : tap::control::Subsystem(&drivers),
      m_desiredOutput(),
      m_pidControllers(),
      m_motors({
          Motor(&drivers, config.leftFrontId,  config.canBus, false, "LF"),
          Motor(&drivers, config.leftBackId,   config.canBus, false, "LB"),
          Motor(&drivers, config.rightBackId,  config.canBus, true,  "RB"),
          Motor(&drivers, config.rightFrontId, config.canBus, true,  "RF")
      }),
      m_logger(drivers.logger),
      m_drivers(&drivers)
{
    for (auto& controller : m_pidControllers)
    {
        controller.setParameter(config.wheelVelocityPidConfig);
    }
}

void HolonomicChassisSubsystem::initialize()
{
    for (auto& motor : m_motors)
    {
        motor.initialize();
    }
}

void HolonomicChassisSubsystem::setWheelVelocities(float leftFront,
                                                   float leftBack,
                                                   float rightBack,
                                                   float rightFront)
{
    leftFront  = mpsToRps(leftFront);
    leftBack   = mpsToRps(leftBack);
    rightBack  = mpsToRps(rightBack);
    rightFront = mpsToRps(rightFront);

    leftFront  = std::clamp(leftFront,  -MAX_CURRENT, MAX_CURRENT);
    leftBack   = std::clamp(leftBack,   -MAX_CURRENT, MAX_CURRENT);
    rightBack  = std::clamp(rightBack,  -MAX_CURRENT, MAX_CURRENT);
    rightFront = std::clamp(rightFront, -MAX_CURRENT, MAX_CURRENT);

    m_desiredOutput[static_cast<uint8_t>(MotorId::LF)] = leftFront;
    m_desiredOutput[static_cast<uint8_t>(MotorId::LB)] = leftBack;
    m_desiredOutput[static_cast<uint8_t>(MotorId::RB)] = rightBack;
    m_desiredOutput[static_cast<uint8_t>(MotorId::RF)] = rightFront;
}

void HolonomicChassisSubsystem::refresh()
{
    ///
    /// @brief uses a wheel's proportional-integral-derivative controller (PID controller)
    /// and desired output to calculate the output current needed to spin the wheel's motor
    /// at the desired speed.
    ///
    /// @param pid the wheel's proportional-integral-derivative controller
    /// @param motor the wheel's motor
    /// @param desiredOutput the wheel's desired output in Rps
    ///
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput, Drivers *m_drivers_lf) -> void
    { 
        if (m_drivers_lf->isEmergencyStopActive()) {
            pid.reset();
            pid.update(0.0f);
        }
        else {
            pid.update(desiredOutput - motor.getEncoder()->getVelocity());
        }
        motor.setDesiredOutput(pid.getValue());
    };

    /**
    * TODO: Power Limiting Logics
    */

    constexpr float CURRENT_RAW_TO_AMPS = 20.0f / 16384.0f;
    constexpr float M3508_TORQUE_CONSTANT = 0.3f;        
    constexpr float M3508_GEAR_RATIO = 3591.0f / 187.0f; 
    constexpr float EFFICIENCY_MULTIPLIER = 1.0f / 0.70f;;        

    float mechPower = 0.0f;

    for (size_t ii = 0; ii < m_motors.size(); ii++)
    {
        float rawCurrent = m_motors[ii].getTorque();
        float rotorVelocityRps = m_motors[ii].getEncoder()->getVelocity();

        float motorCurrentAmps = rawCurrent * CURRENT_RAW_TO_AMPS;
        float shaftTorque = motorCurrentAmps * M3508_TORQUE_CONSTANT;

        float shaftVelocityRps = rotorVelocityRps / M3508_GEAR_RATIO;

        float motorPower = std::abs(shaftTorque * shaftVelocityRps);
        
        mechPower += motorPower;

        runPid(m_pidControllers[ii], m_motors[ii], m_desiredOutput[ii], m_drivers);
    }

    float chassisPower = mechPower * EFFICIENCY_MULTIPLIER;

    m_logger.printf("CHASSIS POWER: %.2f W \n", static_cast<double>(chassisPower));
    m_logger.delay(200);

}

}  // namespace control::chassis
