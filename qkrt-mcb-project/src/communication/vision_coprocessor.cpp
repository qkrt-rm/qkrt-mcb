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

        switch (completeMessage.messageType)
        {
        case JETSON_MESSAGE_TYPE_AIM:
            decodeTurretData(completeMessage);
            break;
        
        case JETSON_MESSAGE_TYPE_NAV:
            
            break;
            
        default:
            break;
        }

    }

    void VisionCoprocessor::initialize()
    {
        drivers->uart.init<VISION_COPROCESSOR_UART_PORT, BAUD_RATE>();
    }

    bool VisionCoprocessor::isOnline() const { return !offlineTimeout.isExpired(); }

    bool VisionCoprocessor::decodeTurretData(const ReceivedSerialMessage& completeMessage)
    {
        if (completeMessage.header.dataLength == sizeof(lastTurretData))
        {
            memcpy(&lastTurretData, &completeMessage.data, sizeof(lastTurretData));

            m_logger.printf("Message Recieved: x=%.3f y= %.3f z=%.3f\n", static_cast<double>(lastTurretData.xPos), static_cast<double>(lastTurretData.yPos), static_cast<double>(lastTurretData.zPos));
            m_logger.delay(200);

            return true;
        }
        return false;
    }

    bool VisionCoprocessor::decodeNavData(const ReceivedSerialMessage& completeMessage)
    {
        if (completeMessage.header.dataLength == sizeof(lastNavData))
        {
            memcpy(&lastNavData, &completeMessage.data, sizeof(lastNavData));

            m_logger.printf("Message Recieved: x=%.3f y= %.3f z=%.3f\n", static_cast<double>(lastNavData.xVel), static_cast<double>(lastNavData.yVel), static_cast<double>(lastNavData.wVel));
            m_logger.delay(200);

            return true;
        }
        return false;
    }

    const TurretData& VisionCoprocessor::getTurretData() const { return lastTurretData; }

    const NavData& VisionCoprocessor::getNavData() const { return lastNavData; }
    
}