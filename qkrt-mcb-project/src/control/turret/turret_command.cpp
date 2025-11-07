#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(Drivers& drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface)
    : _M_turret(turret),
      _M_operatorInterface(operatorInterface),
      _M_pitchSensitivity(1.0f), _M_yawSensitivity(1.0f),
      _M_logger(drivers.logger),
      _M_target(nullptr)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    // using namespace tap::arch;
    
    // static uint32_t prev = clock::getTimeMicroseconds(), curr, dt;
    // static float acc = 0.0f;
    // static const float Speed = 1.0f;

    // curr = clock::getTimeMicroseconds();
    // dt = curr - prev;
    // acc += static_cast<float>(dt * Speed) * 1e-6;
    // prev = curr;

    if (_M_target == nullptr)
    {
        _M_turret.lock();

        float desiredElevation = 0.0f;
        float desiredAzimuth = 0.0f;

        _M_turret.setElevation(desiredElevation);
        _M_turret.setAzimuth(desiredAzimuth);
    }
    else
    {
        _M_turret.unlock();

        float pitchInp = _M_operatorInterface.getTurretPitchInput();
        float yawInp = _M_operatorInterface.getTurretYawInput();

        _M_logger.printf("turret yaw (azimuth): %.3f\n", _M_turret.getAzimuth());
        _M_logger.delay(200);


        _M_turret.setPitchRps(pitchInp);
        _M_turret.setYawRps(yawInp);
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret