#include "turret_command.hpp"

namespace control::turret
{
using communication::TurretData;

TurretCommand::TurretCommand(Drivers& drivers, TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface)
    : _M_turret(turret),
      _M_visionCoprocessor(drivers.visionCoprocessor),
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

    volatile TurretData data = _M_visionCoprocessor.getTurretData();

    const float xConst = 0.0f;
    const float yConst = 0.0f;
    const float zConst = 0.0f;

    float xPos = data.xPos + xConst;
    float yPos = data.yPos + yConst;
    float zPos = data.zPos + zConst;

    float aimAzimuth = std::atan2(xPos, yPos);
    float groundDist = std::hypot(xPos, yPos);
    float aimElevation = std::atan2(zPos, groundDist);

    // aimAzimuth = aimAzimuth * (180.0f / M_PI);

    _M_logger.printf("Message Recieved: Azimuth=%.3f\n", static_cast<double>(aimAzimuth));

    if (true)
    {
        _M_turret.lock();

        float desiredElevation = 0;
        float desiredAzimuth = 0;

        _M_turret.setElevation(desiredElevation);
        _M_turret.setAzimuth(aimAzimuth);
    }
    else
    {
        _M_turret.unlock();

        float pitchInp = _M_operatorInterface.getTurretPitchInput();
        float yawInp = _M_operatorInterface.getTurretYawInput();
        
        // bool is_CV_online = _M_visionCoprocessor.isOnline();

        _M_turret.setPitchRps(pitchInp);
        _M_turret.setYawRps(yawInp);
    }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret