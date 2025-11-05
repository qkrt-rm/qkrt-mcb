#include "vision_coprocessor.hpp"

#include <cassert>

namespace communication::serial
{

VisionCoprocessor::VisionCoprocessor(tap::Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT)
{
    // assert(visionCoprocessorInstance == nullptr);
    // visionCoprocessorInstance = this;
}

void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    switch (completeMessage.messageType)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {
            decodeToTurretAimData(completeMessage);
            return;
        }
    }
}

void VisionCoprocessor::decodeToTurretAimData(const ReceivedSerialMessage& message)
{
    std::memcpy(&_M_lastAimData.pva      , &message.data[0] + offsetof(TurretAimData, pva)      , sizeof(PositionData));
    std::memcpy(&_M_lastAimData.timestamp, &message.data[0] + offsetof(TurretAimData, timestamp), sizeof(uint32_t));
}

}  // namespace communication::serial