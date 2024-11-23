#pragma once

#include <tap/communication/serial/remote.hpp>

namespace control
{

class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::communication::serial::Remote& remote);

    float getChassisLeftVerticalInput() const;

    float getChassisLeftHorizontalInput() const;
private:
    tap::communication::serial::Remote& _M_remote;
};

}  // control