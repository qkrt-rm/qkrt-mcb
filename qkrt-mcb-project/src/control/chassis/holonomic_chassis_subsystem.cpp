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
      m_refdata(&drivers)
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


    // Verify the motors
    // Power Level
    // M3508 motor and C620 driver
    // Idea for getting power leveling
    // Not sure if it should be in setWheelVelocities or refresh
    // tap::communication::sensors ???
    auto m_robotdata = m_refdata->refSerial.getRobotData();
    float m_chassis_power = m_robotdata.chassis.power;
    u_int16_t m_chassis_volt = m_robotdata.chassis.volt;
    u_int16_t m_chassis_current = m_robotdata.chassis.current;
    u_int16_t m_chassis_powerbuffer = m_robotdata.chassis.powerBuffer;
    u_int16_t m_chassis_powerlimit = m_robotdata.chassis.powerConsumptionLimit;
    float scale = 1.0f;
    if (m_chassis_power > m_chassis_powerlimit) {
        scale = m_chassis_powerlimit/m_chassis_power;
    }


    leftFront  = mpsToRpm(leftFront) * scale;
    leftBack   = mpsToRpm(leftBack) * scale;
    rightBack  = mpsToRpm(rightBack) * scale;
    rightFront = mpsToRpm(rightFront) * scale;
    // End of changes


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


    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput) -> void
    {
        pid.update(desiredOutput - motor.getShaftRPM());
        motor.setDesiredOutput(pid.getValue());
    };

    /**
    * TODO: Power Limiting Logic
    */

    for (size_t ii = 0; ii < m_motors.size(); ii++)
    {
        runPid(m_pidControllers[ii], m_motors[ii], m_desiredOutput[ii]);
    }
}

}  // namespace control::chassis
