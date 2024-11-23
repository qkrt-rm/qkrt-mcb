#include "holonomic_chassis_subsystem.hpp"

namespace control::chassis
{

HolonomicChassisSubsystem::HolonomicChassisSubsystem(Drivers& drivers, const ChassisConfig& config)
    : tap::control::Subsystem(&drivers),
      _M_desiredOutput(),
      _M_pidControllers(),
      _M_motors({
          Motor(&drivers, config.leftFrontId,  config.canBus, false, "LF"),
          Motor(&drivers, config.leftBackId,   config.canBus, false, "LB"),
          Motor(&drivers, config.rightFrontId, config.canBus, true,  "RF"),
          Motor(&drivers, config.rightBackId,  config.canBus, true,  "RB")
      })
{
    for (auto& controller : _M_pidControllers)
    {
        controller.setParameter(config.wheelVelocityPidConfig);
    }
}

void HolonomicChassisSubsystem::initialize()
{
    for (auto& motor : _M_motors)
    {
        motor.initialize();
    }
}

void HolonomicChassisSubsystem::setWheelVelocities(float leftFront,
                                          float leftBack,
                                          float rightFront,
                                          float rightBack)
{
    leftFront  = mpsToRpm(leftFront);
    leftBack   = mpsToRpm(leftBack);
    rightFront = mpsToRpm(rightFront);
    rightBack  = mpsToRpm(rightBack);

    leftFront  = std::clamp(leftFront,  -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    leftBack   = std::clamp(leftBack,   -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    rightFront = std::clamp(rightFront, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    rightBack  = std::clamp(rightBack,  -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    _M_desiredOutput[static_cast<uint8_t>(MotorId::LF)] = leftFront;
    _M_desiredOutput[static_cast<uint8_t>(MotorId::LB)] = leftBack;
    _M_desiredOutput[static_cast<uint8_t>(MotorId::RF)] = rightFront;
    _M_desiredOutput[static_cast<uint8_t>(MotorId::RB)] = rightBack;
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

    for (size_t ii = 0; ii < _M_motors.size(); ii++)
    {
        runPid(_M_pidControllers[ii], _M_motors[ii], _M_desiredOutput[ii]);
    }
}

}  // namespace control::chassis
