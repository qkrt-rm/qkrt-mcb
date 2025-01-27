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
    return _M_remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
}

float ControlOperatorInterface::getChassisZInput() const
{
    return _M_remote.getChannel(Remote::Channel::LEFT_VERTICAL);
}

float ControlOperatorInterface::getChassisPitchInput() const
{
    return _M_remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
}

float ControlOperatorInterface::getChassisYawInput() const
{
    return _M_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
}

}  // control
