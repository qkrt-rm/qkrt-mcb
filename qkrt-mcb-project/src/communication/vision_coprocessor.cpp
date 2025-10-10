#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace communication {

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT), drivers(drivers) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {

        offlineTimeout.restart(OFFLINE_TIMEOUT_MS);

        

        // Convert the three floats from little-endian to host-endian
        float x, y, z;
        std::memcpy(&x, payload, sizeof(float));
        std::memcpy(&y, payload + sizeof(float), sizeof(float));
        std::memcpy(&z, payload + 2 * sizeof(float), sizeof(float));
    }

    void VisionCoprocessor::initialize()
    {
        drivers->uart.init<VISION_COPROCESSOR_UART_PORT, BAUD_RATE>();
    }

}