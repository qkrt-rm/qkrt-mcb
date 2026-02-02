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
            
            // teriatry operator syntax below
            m_chassisWInput = 
                m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) > 0 ? 0.25f :
                m_remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) < 0 ? 0.0f :
                m_chassisWInput;

            m_turretPitchInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_VERTICAL), -1.0f, 1.0f);
            m_turretYawInput = std::clamp(m_remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);

            m_flywheelInput = m_remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
            m_agitatorInput = m_remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
        break;

        case DeviceType::KEYBOARD_MOUSE:

            speedBoost = m_remote.keyPressed(Remote::Key::SHIFT);
            
            // chassis X Input
            m_chassisXInput = 
                m_remote.keyPressed(Remote::Key::W) && m_remote.keyPressed(Remote::Key::S) ? 0.0f:
                m_remote.keyPressed(Remote::Key::W) && speedBoost ? 2.0:
                m_remote.keyPressed(Remote::Key::S) && speedBoost ? 2.0:
                m_remote.keyPressed(Remote::Key::W) ? 1.0f : 
                m_remote.keyPressed(Remote::Key::S) ? 1.0f :
                0.0f;         

            // chassis Y Input
            m_chassisYInput = 
                m_remote.keyPressed(Remote::Key::A) && m_remote.keyPressed(Remote::Key::D) ? 0.0f:
                m_remote.keyPressed(Remote::Key::A) && speedBoost ? 2.0:
                m_remote.keyPressed(Remote::Key::D) && speedBoost ? 2.0:
                m_remote.keyPressed(Remote::Key::A) ? 1.0f : 
                m_remote.keyPressed(Remote::Key::D) ? 1.0f :
                0.0f;


            // chassis W Input (triggering beyblade ) (maybe mouse wheel)


            // turret Pitch Input (int16_t: range of values from -32,768 to 32,767)
            m_turretPitchInput = std::clamp<int>(m_remote.getMouseX(),-32768, 32767); // TODO: tweak clamp

            // turret Yaw Input 
            m_turretYawInput = std::clamp<int>(m_remote.getMouseY(), -32768, 32767); // TODO: tweak clamp
        
            // flywheel Input
            m_flywheelInput = m_remote.getMouseL();
            
            // agigtator Input
            //m_agitatorInput = m_remote.getMouseR();


        break;

    }
}

}  // control
