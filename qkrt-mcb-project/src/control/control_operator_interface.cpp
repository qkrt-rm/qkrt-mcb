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
                m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) > 0 ? 0.25f :
                m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) < 0 ? 0.0f :
                m_chassisWInput;

            m_turretPitchInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
            m_turretYawInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);
            
            m_agitatorInput = m_remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;


    }
}

}  // control
