#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace communication {

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT), drivers(drivers) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {
        //TODO: Switchcase based on message type, seperate decode func
        offlineTimeout.restart(OFFLINE_TIMEOUT_MS);

        if (completeMessage.header.dataLength == sizeof(lastTurretData))
        {
            memcpy(&lastTurretData, &completeMessage.data, sizeof(lastTurretData));
            _M_logger.printf("Message Recieved: x=%.3f y= %.3f z=%.3f\n", lastTurretData.xPos, lastTurretData.yPos, lastTurretData.zPos);
        }

    }

    void VisionCoprocessor::initialize()
    {
        drivers->uart.init<VISION_COPROCESSOR_UART_PORT, BAUD_RATE>();
    }

    bool VisionCoprocessor::isOnline() const { return !offlineTimeout.isExpired(); }

    const TurretData& VisionCoprocessor::getTurretData() const { return lastTurretData; }
    
}