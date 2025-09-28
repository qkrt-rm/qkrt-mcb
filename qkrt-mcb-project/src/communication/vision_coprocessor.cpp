#include "vision_coprocessor.hpp"

namespace tap::communication::serial 
{

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers, Uart::UartPort port, bool isRxCRCEnforcementEnabled)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT, isRxCRCEnforcementEnabled) {}

    void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
    {
        
    }


}