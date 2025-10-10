#pragma once

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"


namespace tap::communication::serial
{
    class VisionCoprocessor : public DJISerial 
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

        private:
        
        Drivers* drivers;

            static constexpr Uart::UartPort VISION_COPROCESSOR_UART_PORT = Uart::UartPort::Uart1;
        static constexpr uint32_t BAUD_RATE = 115200;
        static constexpr uint16_t OFFLINE_TIMEOUT_MS = 2000;
        tap::arch::MilliTimeout offlineTimeout;

    };
}

