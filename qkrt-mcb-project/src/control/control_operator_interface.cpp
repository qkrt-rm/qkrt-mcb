#include "control_operator_interface.hpp"

using tap::communication::serial::Remote;

namespace control
{

ControlOperatorInterface::ControlOperatorInterface(tap::communication::serial::Remote& remote)
    : _M_remote(remote)
{
}

float ControlOperatorInterface::getChassisXInput() const
{
    return std::clamp(_M_remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), -1.0f, 1.0f);
}

float ControlOperatorInterface::getChassisZInput() const
{
    return std::clamp(_M_remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f);
}

float ControlOperatorInterface::getTurretPitchInput() const
{
    return std::clamp(_M_remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
}

float ControlOperatorInterface::getTurretYawInput() const
{
    return std::clamp(_M_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);
}




/// Adding Flywheel Input
bool ControlOperatorInterface::getFlyWheelInput() {
    return _M_remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
}

// Adding Agitator Input
bool ControlOperatorInterface::getAgitatorInput() {
    return _M_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;

}

}  // control
