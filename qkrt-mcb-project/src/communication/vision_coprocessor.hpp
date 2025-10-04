#pragma once

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"


namespace tap::communication::serial
{
    class VisionCoprocessor : public DJISerial 
    {
        public:
            static constexpr Uart::UartPort VISION_COPROCESSOR_UART_PORT = Uart::UartPort::Uart1;
            /**
            * Construct MyDJISerial.
            *
            * @param[in] drivers Pointer to TAP drivers.
            * @param[in] port UART port to use.
            * @param[in] crcEnforcement Enable/disable RX CRC enforcement.
            */
            VisionCoprocessor(Drivers* drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled);

        protected:
            /**
            * Callback executed when a full message has been received.
            *
            * @param[in] completeMessage Reference to the received message.
            */

            void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;


    };
}

