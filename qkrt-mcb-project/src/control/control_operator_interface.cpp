#include "control_operator_interface.hpp"

using tap::communication::serial::Remote;

namespace control
{

ControlOperatorInterface::ControlOperatorInterface(tap::communication::serial::Remote& remote)
    : m_remote(remote)
{
}

//TODO poll for toggle switch between keyboard and remote

void ControlOperatorInterface::pollInputDevices()
{
    handleToggles();

    switch (m_activeDevice)
    {
        case DeviceType::CONTROLLER:
            pollController();
            break;

        case DeviceType::KEYBOARD_MOUSE:
            pollKeyboardMouse();
            break;
    }
}

void ControlOperatorInterface::handleToggles()
{
    if (m_remote.keyPressed(Remote::Key::C))
    {
        if(!m_toggledDevice)
        {
            m_activeDevice = (m_activeDevice == DeviceType::CONTROLLER) ? 
                                DeviceType::KEYBOARD_MOUSE : DeviceType::CONTROLLER;
            m_toggledDevice = true; 
        }
    }
    else 
    { 
        m_toggledDevice = false; 
    }

    //TODO: agitator reversal
}

void ControlOperatorInterface::pollController()
{
    m_chassisXInput = std::clamp(m_remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f);
    m_chassisYInput = std::clamp(m_remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), -1.0f, 1.0f);
    m_beybladeDirection = 
        m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) > 0 ? true :
        m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) < 0 ? false :
        m_beybladeDirection;

    m_turretPitchInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
    m_turretYawInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);

    m_autoAimInput =  m_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN;
    m_autoNavInput = m_remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;

}

void ControlOperatorInterface::pollKeyboardMouse()
{
    float moveSpeedMultiplier = m_remote.keyPressed(Remote::Key::SHIFT) ? 4.0f : 1.5f;
    
    m_turretPitchInput = -normalizeMouseTanh(m_remote.getMouseY());
    m_turretYawInput = normalizeMouseTanh(m_remote.getMouseX());

    // WASD mapping
    float rawX = (m_remote.keyPressed(Remote::Key::W) - m_remote.keyPressed(Remote::Key::S));
    float rawY = (m_remote.keyPressed(Remote::Key::D) - m_remote.keyPressed(Remote::Key::A));

    Vector2f rawMoveInput(rawX, rawY);
    float rawInputLen = rawMoveInput.getLength();

    constexpr float KEYBOARD_ACCEL = 3.0f;  
    constexpr float KEYBOARD_DECEL = 4.0f;  
    constexpr float DT = 0.002f;            

    Vector2f currentMove(m_chassisXInput, m_chassisYInput);

    if (rawInputLen > 0.0f)
    {
        Vector2f moveDir = rawMoveInput / rawInputLen;      //normalize
        currentMove += moveDir * KEYBOARD_ACCEL * DT;       //acclerate when holding
    }
    else
    {
        float len = currentMove.getLength();
        if (len > 0.0f)
        {
            float decelerationFactor = 1.0f - (KEYBOARD_DECEL * DT / len);
            currentMove *= std::max(decelerationFactor, 0.0f);
        }
    }

    float currentLen = currentMove.getLength();
    if (currentLen > moveSpeedMultiplier)
    {
        currentMove = (currentMove / currentLen) * moveSpeedMultiplier;
    }

    m_chassisXInput = currentMove.x;
    m_chassisYInput = currentMove.y;

    //beyblade
    bool currEPressed = m_remote.keyPressed(Remote::Key::E);
    bool currQPressed = m_remote.keyPressed(Remote::Key::Q);

    if (currEPressed && !m_prevEState) 
    {
        m_beybladeDirection = (m_beybladeDirection == 1.0f) ? 0.0f : 1.0f;
    }
    else if (currQPressed && !m_prevQState) 
    {
        m_beybladeDirection = (m_beybladeDirection == -1.0f) ? 0.0f : -1.0f;
    }

    m_prevEState = currEPressed;
    m_prevQState = currQPressed;
    }

}  // control
