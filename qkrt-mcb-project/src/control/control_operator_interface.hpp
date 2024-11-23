#pragma once

#include <tap/communication/serial/remote.hpp>

namespace control
{

class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::communication::serial::Remote& remote);
    DISALLOW_COPY_AND_ASSIGN(ControlOperatorInterface)
    ~ControlOperatorInterface() = default;

    /**
     * @return The normalized value used for left and right chassis movement.
     */
    float getChassisXInput() const;

    /**
     * @return The normalized value used for forward and backward chassis movement.
     */
    float getChassisZInput() const;

    /**
     * @return The normalized value used for chassis rotation.
     */
    float getChassisRInput() const;
private:
    tap::communication::serial::Remote& _M_remote;
};

}  // control