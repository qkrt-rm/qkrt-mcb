#include "control_operator_interface.hpp"

using tap::communication::serial::Remote;

namespace control
{

ControlOperatorInterface::ControlOperatorInterface(tap::communication::serial::Remote& remote)
    : m_remote(remote), m_activeDevice(DeviceType::CONTROLLER)
{
}

//TODO poll for toggle switch between keyboard and remote

void ControlOperatorInterface::pollInputDevices()
{
    switch (m_activeDevice)
    {
        case DeviceType::CONTROLLER:
        
            m_chassisXInput = std::clamp(m_remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f);
            m_chassisYInput = std::clamp(m_remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), -1.0f, 1.0f);
            m_chassisWInput = 
                m_remote.getWheel() > WHEEL_DEADZONE ? 0.25f :
                m_remote.getWheel() < -WHEEL_DEADZONE ? 0.0f :
                m_chassisWInput;

            m_turretPitchInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
            m_turretYawInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);

            m_flywheelInput = m_remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
            m_agitatorInput = m_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
            m_emergencystopInput = m_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN;


<<<<<<< HEAD
    }
=======


// Adding Flywheel Input
bool ControlOperatorInterface::getFlyWheelInput() {
    return _M_remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
}

// Adding Agitator Input
bool ControlOperatorInterface::getAgitatorInput() {
    return _M_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
}

// Adding Emergency Stop Input
bool ControlOperatorInterface::getEmergencyStopInput() {
    return false;
    // Find a button on the remote for it.
    //return _M_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
>>>>>>> 6081a8c (Starting work emergency stop)
}

}  // control
