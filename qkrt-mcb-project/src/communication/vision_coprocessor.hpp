#pragma once

#include <tap/communication/serial/dji_serial.hpp>
#include <tap/communication/serial/uart.hpp>
#include <tap/architecture/timeout.hpp>

#include "jetson_message.hpp"
#include "communication/logger/logger.hpp"

class Drivers;

namespace control::chassis { class HolonomicChassisSubsystem; }
namespace control::turret { class TurretSubsystem; }

namespace communication
{
    using tap::communication::serial::Uart;

    class VisionCoprocessor : public tap::communication::serial::DJISerial
    {
        public:
   
            VisionCoprocessor(Drivers* drivers);

            void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

            void initialize();

            bool decodeTurretData(const ReceivedSerialMessage& completeMessage);

            bool decodeNavData(const ReceivedSerialMessage& completeMessage);

            const TurretData& getTurretData() const;

            const NavData& getNavData() const;

            void sendData();

            void sendOdomData();

            void setChassisSubsystem(control::chassis::HolonomicChassisSubsystem* chassis) { m_chassis = chassis; }
            void setTurretSubsystem(control::turret::TurretSubsystem* turret) { m_turret = turret; }

            bool isOnline() const;


        private:
            static constexpr Uart::UartPort VISION_COPROCESSOR_UART_PORT = Uart::UartPort::Uart1;

            static constexpr uint32_t BAUD_RATE = 115200;

            static constexpr uint16_t OFFLINE_TIMEOUT_MS = 2000;

            tap::arch::MilliTimeout offlineTimeout;

            logger::Logger& m_logger;         //member that references logger object

            TurretData lastTurretData;          //struct with turret data from jetson

            NavData lastNavData;

            tap::communication::sensors::imu::bmi088::Bmi088& m_imu;

            control::chassis::HolonomicChassisSubsystem* m_chassis = nullptr;
            control::turret::TurretSubsystem* m_turret = nullptr;

    };
}

