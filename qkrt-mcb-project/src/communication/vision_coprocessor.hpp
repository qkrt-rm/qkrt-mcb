#pragma once

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "logger.hpp" // Ensure this header defines the Logger class or type

#include "aim_message.hpp"


namespace src
{
class Drivers;
}
namespace communication
{
    using tap::communication::serial::Uart;
    class VisionCoprocessor : public tap::communication::serial::DJISerial
    {
        public:
            /**
            * Construct MyDJISerial.
            *
            * @param[in] drivers Pointer to TAP drivers.
            * @param[in] port UART port to use.
            * @param[in] crcEnforcement Enable/disable RX CRC enforcement.
            */
            VisionCoprocessor(Drivers* drivers);

            /**
            * Callback executed when a full message has been received.
            *
            * @param[in] completeMessage Reference to the received message.
            */

            void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

            void initialize();

            const TurretData& getTurretData() const;

        private:
        
            Drivers* drivers;

            static constexpr Uart::UartPort VISION_COPROCESSOR_UART_PORT = Uart::UartPort::Uart1;
            static constexpr uint32_t BAUD_RATE = 115200;

            communication::serial::Logger & _M_logger;

            TurretData lastTurretData;

    };
}

