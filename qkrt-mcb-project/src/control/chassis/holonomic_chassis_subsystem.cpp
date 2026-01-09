#include "holonomic_chassis_subsystem.hpp"

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
      m_current_sensor(UNKOWN_WHAT_TO_INITIALIZE_WITH),
      m_power_limiter(&drivers, m_current_sensor, m_power_limiter_startingEnergyBuffer, m_power_limiter_energyBufferCritThreshold, m_power_limiter_energyBufferLimitThreshold),
      m_drivers(&drivers),
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
    leftFront  = mpsToRpm(leftFront);
    leftBack   = mpsToRpm(leftBack);
    rightBack  = mpsToRpm(rightBack);
    rightFront = mpsToRpm(rightFront);

    leftFront  = std::clamp(leftFront,  -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    leftBack   = std::clamp(leftBack,   -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    rightBack  = std::clamp(rightBack,  -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    rightFront = std::clamp(rightFront, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

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
    /// @param desiredOutput the wheel's desired output in Rpm
    ///
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput, Drivers *m_drivers_lf) -> void
    { 
        if (m_drivers_lf->isEmergencyStopActive()) {
            pid.reset();
            pid.update(0.0f);
        }
        else {
            pid.update(desiredOutput - motor.getShaftRPM());
        }
        motor.setDesiredOutput(pid.getValue());
    };

    /**
    * TODO: Power Limiting Logic
    */

    for (size_t ii = 0; ii < m_motors.size(); ii++)
    {
        runPid(m_pidControllers[ii], m_motors[ii], m_desiredOutput[ii],m_drivers);
    }
}

}  // namespace control::chassis
