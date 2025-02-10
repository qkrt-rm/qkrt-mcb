#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface,
                             Uart& uart)
    : _M_turret(turret),
      _M_operatorInterface(operatorInterface),
      _M_uart(uart),
      _M_pitchSensitivity(1.0f), _M_yawSensitivity(1.0f),
      _M_aimAssist(false)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

void TurretCommand::execute()
{
    using namespace tap::arch;

    static uint32_t prev = clock::getTimeMicroseconds(), curr, dt;
    static float acc = 0.0f;
    static const float Speed = 1.0f;

    curr = tap::arch::clock::getTimeMicroseconds();
    dt = curr - prev;
    acc += static_cast<float>(dt * Speed) * 1e-6;
    prev = curr;

    if (false /* _M_target */)
    {
        _M_turret.lock();

        float desiredElevation = 0.0f;
        float desiredAzimuth = std::floor(acc) * M_PI;

        _M_turret.setElevation(desiredElevation);
        _M_turret.setAzimuth(desiredAzimuth);
    }
    else
    {
        _M_turret.unlock();

        float pitchInp = _M_operatorInterface.getTurretPitchInput();
        float yawInp = cos(acc);
    
        _M_turret.setPitchRps(pitchInp);
        _M_turret.setYawRps(yawInp);
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret