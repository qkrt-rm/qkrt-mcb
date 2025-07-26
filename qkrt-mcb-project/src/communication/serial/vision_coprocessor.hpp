#pragma once

#include <tap/communication/serial/dji_serial.hpp>

namespace tap
{
class Drivers;
}  // namespace tap

namespace communication::serial
{

class VisionCoprocessor : public tap::communication::serial::DJISerial
{
public:
    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_UART_PORT =
            tap::communication::serial::Uart::UartPort::Uart1;

    struct PositionData
    {
        float xPos;  ///< x position of the target (in m).
        float yPos;  ///< y position of the target (in m).
        float zPos;  ///< z position of the target (in m).

        float xVel;  ///< x velocity of the target (in m/s).
        float yVel;  ///< y velocity of the target (in m/s).
        float zVel;  ///< z velocity of the target (in m/s).

        float xAcc;  ///< x acceleration of the target (in m/s^2).
        float yAcc;  ///< y acceleration of the target (in m/s^2).
        float zAcc;  ///< z acceleration of the target (in m/s^2).
    };

    struct TurretAimData
    {
        PositionData pva;
        uint32_t timestamp;  ///< timestamp in microseconds
    };

    explicit VisionCoprocessor(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(VisionCoprocessor);
    ~VisionCoprocessor() = default;

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    inline const TurretAimData& getLastAimData(uint8_t turretID) const
    {
        return _M_lastAimData;
    }

    void decodeToTurretAimData(const ReceivedSerialMessage& message);

private:
    enum TxMessageTypes : uint16_t
    {

    };

    enum RxMessageTypes : uint16_t
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 1
    };

    static VisionCoprocessor* visionCoprocessorInstance;

    TurretAimData _M_lastAimData;
};

}  // namespace communication::serial