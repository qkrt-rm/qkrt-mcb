#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace communication {

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT), drivers(drivers), _M_logger(drivers->logger) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {

        

        // Convert the three floats from little-endian to host-endian
        if (completeMessage.header.dataLength == sizeof(lastTurretData))
        {
            memcpy(&lastTurretData, &completeMessage.data, sizeof(lastTurretData));
        }

        // Extract the payload
        // const uint8_t* payload = completeMessage.data;

        // float x, y, z;
        // std::memcpy(&x, payload, sizeof(float));
        // std::memcpy(&y, payload + sizeof(float), sizeof(float));
        // std::memcpy(&z, payload + 2 * sizeof(float), sizeof(float));
    }

    void VisionCoprocessor::initialize()
    {
        drivers->uart.init<VISION_COPROCESSOR_UART_PORT, BAUD_RATE>();
    }

}