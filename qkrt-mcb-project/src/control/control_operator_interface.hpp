#pragma once

#include <tap/communication/serial/remote.hpp>

namespace tap
{
class Drivers;
}  // namespace tap

namespace control
{

class ControlOperatorInterface
{
    using Remote = tap::communication::serial::Remote;

public:
    ControlOperatorInterface(tap::Drivers* drivers);
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
     * @return The normalized value used for turret pitch adjustments.
     */
    float getTurretPitchInput() const;

    /**
     * @return The normalized value used for turret yaw adjustments.
     */
    float getTurretYawInput() const;
    
private:
    Remote& _M_remote;
};

}  // namespace control