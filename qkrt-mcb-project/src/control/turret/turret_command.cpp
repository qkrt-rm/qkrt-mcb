#include "turret_command.hpp"

namespace control::turret
{

TurretCommand::TurretCommand(TurretSubsystem& turret,
                             ControlOperatorInterface& operatorInterface,
                             Uart& uart)
    : _M_turret(turret),
      _M_operatorInterface(operatorInterface),
      _M_uart(uart),
      _M_pitchSensitivity(1.0f), _M_yawSensitivity(1.0f)
{
    addSubsystemRequirement(&turret);
}


void TurretCommand::initialize()
{
}

// namespace qkrt::cv
// {
//     struct packet
//     {
//         float pitch;
//         float yaw;
//         uint8_t checksum;
//     };

//     template<typename T>
//     inline constexpr uint8_t* addr(T& _v)
//         { return reinterpret_cast<uint8_t*>(&_v); }

//     template<typename T>
//     inline constexpr size_t size(const T& _v)
//         { return sizeof(_v); }

//     inline bool validate(const packet& _pkt)
//         { return (static_cast<uint8_t>(_pkt.pitch + _pkt.yaw) & 0xff)
//                   == _pkt.checksum; }
// }

void TurretCommand::execute()
{
    // qkrt::cv::packet packet;
    // size_t nBytes = _M_uart.read(Uart::UartPort::Uart1,
    //                              qkrt::cv::addr(packet),
    //                              qkrt::cv::size(packet));

    // if (nBytes && qkrt::cv::validate(packet))
    // {
    //     _M_turret.setPitch(packet.pitch);
    //     _M_turret.setPitch(packet.yaw);
    // }
    // else
    // {
    float pitch = _M_operatorInterface.getChassisPitchInput();
    float yaw = _M_operatorInterface.getChassisYawInput();
    _M_turret.setPitch(pitch);
    _M_turret.setYaw(yaw);
    // }
}

void TurretCommand::end(bool /* interrupted */)
{
}

}  // namespace control::turret