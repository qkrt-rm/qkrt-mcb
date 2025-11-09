#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"
# include "drivers.hpp"

namespace communication {

    VisionCoprocessor::VisionCoprocessor(tap::Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT), drivers(drivers) {} //_M_logger(&static_cast<Drivers*>(drivers)->logger) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {
        //TODO: Switchcase based on message type, seperate decode func
        offlineTimeout.restart(OFFLINE_TIMEOUT_MS);

        //if (completeMessage.header.dataLength == sizeof(lastTurretData))
        
        memcpy(&lastTurretData, &completeMessage.data, sizeof(lastTurretData));

        lastTurretData.xPos = 80.51;
        float y = lastTurretData.yPos;
        float z = lastTurretData.zPos;

            //_M_logger.printf("Message Recieved: x=%.3f y= %.3f z=%.3f\n", static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
            //_M_logger->printf("HEllio");
            //_M_logger->delay(200);

    }

    void VisionCoprocessor::initialize()
    {
        drivers->uart.init<VISION_COPROCESSOR_UART_PORT, BAUD_RATE>();
    }

    bool VisionCoprocessor::isOnline() const { return !offlineTimeout.isExpired(); }

    const TurretData& VisionCoprocessor::getTurretData() const { return lastTurretData; }
    
}