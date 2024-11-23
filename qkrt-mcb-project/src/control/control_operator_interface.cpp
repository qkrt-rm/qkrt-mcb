#include "control_operator_interface.hpp"

using tap::communication::serial::Remote;

namespace control
{

ControlOperatorInterface::ControlOperatorInterface(tap::communication::serial::Remote& remote)
    : _M_remote(remote)
{
}

float ControlOperatorInterface::getChassisLeftVerticalInput() const
{
    return _M_remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
}

float ControlOperatorInterface::getChassisLeftHorizontalInput() const
{
    return _M_remote.getChannel(Remote::Channel::LEFT_VERTICAL);
}

}  // control
