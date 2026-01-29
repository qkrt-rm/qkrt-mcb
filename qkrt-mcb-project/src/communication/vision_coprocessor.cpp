#include "drivers.hpp"
#include "vision_coprocessor.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "control/chassis/holonomic_chassis_subsystem.hpp"

namespace communication {

    VisionCoprocessor::VisionCoprocessor(Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_UART_PORT)
    , m_logger(drivers->logger)
    , m_imu(drivers->bmi088) {}

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
            decodeNavData(completeMessage);
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

    void VisionCoprocessor::sendData()
    {
        sendOdomData();
    }

    void VisionCoprocessor::sendOdomData()
    {
        DJISerial::SerialMessage<sizeof(OdomData)> message;
        message.messageType = MCB_MESSAGE_TYPE_ODOM;

        OdomData* data = reinterpret_cast<OdomData*>(message.data);

        // Chassis wheel encoder velocities
        if (m_chassis != nullptr)
        {
            data->wheelLF = m_chassis->getWheelVelocity(0);
            data->wheelLB = m_chassis->getWheelVelocity(1);
            data->wheelRB = m_chassis->getWheelVelocity(2);
            data->wheelRF = m_chassis->getWheelVelocity(3);
        }
        else
        {
            data->wheelLF = data->wheelLB = data->wheelRB = data->wheelRF = 0.0f;
        }

        // Turret encoder positions and velocities
        if (m_turret != nullptr)
        {
            data->turretYawPos = m_turret->getAzimuth();
            data->turretYawVel = m_turret->getYawVelocity();
            data->turretPitchPos = m_turret->getElevation();
            data->turretPitchVel = m_turret->getPitchVelocity();
        }
        else
        {
            data->turretYawPos = data->turretYawVel = 0.0f;
            data->turretPitchPos = data->turretPitchVel = 0.0f;
        }

        // IMU data
        data->imuAx = m_imu.getAx();
        data->imuAy = m_imu.getAy();
        data->imuAz = m_imu.getAz();
        data->imuGx = m_imu.getGx();
        data->imuGy = m_imu.getGy();
        data->imuGz = m_imu.getGz();
        data->imuYaw = m_imu.getYaw();
        data->imuPitch = m_imu.getPitch();
        data->imuRoll = m_imu.getRoll();

        message.setCRC16();
        drivers->uart.write(VISION_COPROCESSOR_UART_PORT, reinterpret_cast<uint8_t*>(&message), sizeof(message));
    }
    
    //TOD0: SEND COLOUR DATA

}