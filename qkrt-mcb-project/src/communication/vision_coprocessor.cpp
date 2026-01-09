#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"
# include "drivers.hpp"

namespace communication {

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT)
    , m_logger(drivers->logger) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {
        //TODO: Switchcase based on message type, seperate decode func
        offlineTimeout.restart(OFFLINE_TIMEOUT_MS);

        if (completeMessage.header.dataLength == sizeof(lastTurretData))
        {
            memcpy(&lastTurretData, &completeMessage.data, sizeof(lastTurretData));

            // float x = lastTurretData.xPos;
            // float y = lastTurretData.yPos;
            // float z = lastTurretData.zPos;

            // //m_logger.printf("Message Recieved: x=%.3f y= %.3f z=%.3f\n", static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
            // m_logger.printf("Message Recieved: Y=%.3f\n", static_cast<double>(y));
            // m_logger.delay(400);
        }
    }

    void VisionCoprocessor::initialize()
    {
        drivers->uart.init<VISION_COPROCESSOR_UART_PORT, BAUD_RATE>();
    }

    bool VisionCoprocessor::isOnline() const { return !offlineTimeout.isExpired(); }

    const TurretData& VisionCoprocessor::getTurretData() const { return lastTurretData; }
    
}