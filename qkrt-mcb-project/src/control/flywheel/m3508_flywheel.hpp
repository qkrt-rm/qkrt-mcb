#pragma once

#include "drivers.hpp"
#include "flywheel_subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    #include "tap/mock/dji_motor_mock.hpp"
#else
    #include "tap/motor/dji_motor.hpp"
#endif

#include <tap/control/subsystem.hpp>
#include "tap/util_macros.hpp"

#include <array>
#include <cstdint>

namespace control::flywheel 
{
    struct M3508FlywheelConfig
    {
        tap::motor::MotorId leftFlyWheelId;
        tap::motor::MotorId rightFlywheelId;
        tap::can::CanBus canBus;
        modm::Pid<float>::Parameter wheelVelocityPidConfig;
    };
    

    class M3508FlywheelSubsystem : public FlywheelSubsystem
    {
        private:
            enum class MotorId : uint8_t //change later 
            {
                LFly = 0,
                RFly,
                NUM_MOTORS,
            };

            static constexpr float MAX_WHEELSPEED_RPM = 10000.0f; //change maybe

            using Pid = modm::Pid<float>;
            using Motor = tap::motor::DjiMotor; //use a simplfied name for a complex datatype
        public:
            M3508FlywheelSubsystem(Drivers& drivers, const M3508FlywheelConfig& config);

            void initialize() override;

            void setTargetSpeed(float speed) override;

            void refresh() override;

            const char* getName() override { return "DJIFlywheel"; }

        private: 
            std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_desiredOutput;
            std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_motors;
            std::array<Pid,   static_cast<uint8_t>(MotorId::NUM_MOTORS)> m_pidControllers;
            Drivers* m_drivers;
    };


}  // namespace control::flywheel