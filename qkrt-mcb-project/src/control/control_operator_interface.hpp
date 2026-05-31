#pragma once

#include <tap/communication/serial/remote.hpp>

namespace control
{

class ControlOperatorInterface
{
public:
    enum class DeviceType{
        CONTROLLER,
        KEYBOARD_MOUSE,
    };

    enum class RobotType{
        STANDARD,
        HERO,
        SENTRY,
    };

    
    ControlOperatorInterface(tap::communication::serial::Remote& remote);
    DISALLOW_COPY_AND_ASSIGN(ControlOperatorInterface)
    ~ControlOperatorInterface() = default;

    void pollInputDevices ();

    //chassis outputs
    float getChassisXInput() const { return m_chassisXInput; };
    float getChassisYInput() const { return m_chassisYInput; };
    float getChassisBeyblade() const { return m_beybladeDirection; }; 

    //turret outputs
    float getTurretPitchInput() const { return m_turretPitchInput; };
    float getTurretYawInput() const { return m_turretYawInput; };

    bool isAutoAim () const { return m_autoAimInput; };
    
private:
    void pollController();
    void pollKeyboardMouse();
    void handleToggles();

    //dependencies
    tap::communication::serial::Remote& m_remote;
    RobotType m_robot;

    //config states
    DeviceType m_activeDevice = DeviceType::CONTROLLER;
    bool m_toggledDevice = false;
    bool m_autoAimInput = false;

    //control states
    float m_chassisXInput = 0.0f;
    float m_chassisYInput = 0.0f;
    float m_beybladeDirection = 0.0f;
    bool m_prevEState = false;       
    bool m_prevQState = false;

    float m_turretPitchInput = 0.0f;
    float m_turretYawInput = 0.0f;

};

inline float normalizeMouseTanh(int16_t rawMouseVal)
{
    constexpr float SENSITIVITY_SCALER = 0.01f;     
    return std::tanh(static_cast<float>(rawMouseVal) * SENSITIVITY_SCALER);
}

}  // control