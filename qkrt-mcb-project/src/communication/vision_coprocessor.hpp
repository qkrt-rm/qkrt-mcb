#pragma once

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "logger.hpp" 
#include "aim_message.hpp"


namespace tap
{
class Drivers;
}
namespace communication
{
    using tap::communication::serial::Uart;
    class VisionCoprocessor : public tap::communication::serial::DJISerial
    {
        public:
   
            VisionCoprocessor(tap::Drivers* drivers);


            void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

            void initialize();

            const TurretData& getTurretData() const;
            
            bool isOnline() const;  


        private:

        
            tap::Drivers* drivers;

            static constexpr Uart::UartPort VISION_COPROCESSOR_UART_PORT = Uart::UartPort::Uart6;

            static constexpr uint32_t BAUD_RATE = 115200;

            static constexpr uint16_t OFFLINE_TIMEOUT_MS = 2000;

            tap::arch::MilliTimeout offlineTimeout;

           // serial::Logger & _M_logger;         //member that references logger object

            TurretData lastTurretData;          //struct with turret data from jetson

    };
}

