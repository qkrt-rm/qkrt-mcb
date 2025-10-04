#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace tap::communication::serial {

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT, isRxCRCEnforcementEnabled) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {

        // Extract the payload
        const uint8_t* payload = completeMessage.data;

        // Convert the three floats from little-endian to host-endian
        float x, y, z;
        std::memcpy(&x, payload, sizeof(float));
        std::memcpy(&y, payload + sizeof(float), sizeof(float));
        std::memcpy(&z, payload + 2 * sizeof(float), sizeof(float));
    }
}