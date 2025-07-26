#include "control_operator_interface.hpp"

#include <tap/drivers.hpp>

namespace control
{

ControlOperatorInterface::ControlOperatorInterface(tap::Drivers* drivers)
    : _M_remote(drivers->remote)
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

}  // control
