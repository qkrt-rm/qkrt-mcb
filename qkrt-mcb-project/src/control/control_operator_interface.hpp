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

    void pollInputDevices ();

    float getChassisXInput() const { return m_chassisXInput; };

    float getChassisYInput() const { return m_chassisYInput; };

    float getChassisWInput() const { return m_chassisWInput; }; 

    float getTurretPitchInput() const { return m_turretPitchInput; };

    float getTurretYawInput() const { return m_turretYawInput; };

    bool getFlywheelInput() const { return m_flywheelInput; };

    bool getAgitatorInput() const { return m_agitatorInput; };




    
private:
    tap::communication::serial::Remote& m_remote;

    enum class DeviceType{
        CONTROLLER,
        KEYBOARD_MOUSE,
    } m_activeDevice;

    float m_chassisXInput = 0.0f;
    float m_chassisYInput = 0.0f;
    float m_chassisWInput = 0.0f;

    float m_turretPitchInput = 0.0f;
    float m_turretYawInput = 0.0f;

    bool m_flywheelInput = false;
    bool m_agitatorInput = false;
    bool speedBoost = false;

};

}  // control