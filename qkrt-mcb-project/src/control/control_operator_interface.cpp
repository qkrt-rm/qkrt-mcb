#include "control_operator_interface.hpp"

using tap::communication::serial::Remote;

namespace control
{

ControlOperatorInterface::ControlOperatorInterface(tap::communication::serial::Remote& remote, RobotType robot)
    : m_remote(remote), m_robot(robot)
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
    m_isChassisBey = 
        m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) > 0 ? true :
        m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) < 0 ? false :
        m_isChassisBey;

    m_turretPitchInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
    m_turretYawInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);

    m_autoAimInput =  m_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN;
}

void ControlOperatorInterface::pollKeyboardMouse()
{
    float moveSpeedMultiplier = m_remote.keyPressed(Remote::Key::SHIFT) ? 4.0f : 1.5f;

    // WASD mapping
    m_chassisXInput = (m_remote.keyPressed(Remote::Key::W) - m_remote.keyPressed(Remote::Key::S)) * moveSpeedMultiplier;
    m_chassisYInput = (m_remote.keyPressed(Remote::Key::D) - m_remote.keyPressed(Remote::Key::A)) * moveSpeedMultiplier;

    // mouse mapping
    m_turretPitchInput = -normalizeMouseTanh(m_remote.getMouseY());
    m_turretYawInput = normalizeMouseTanh(m_remote.getMouseX());

    m_isChassisBey = (m_remote.keyPressed(Remote::Key::E)) 
}

}  // control
