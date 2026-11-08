#pragma once
#include <modm/platform.hpp>
#include <modm/processing/protothread.hpp>
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/voltage/voltage_sensor_interface.hpp"
#include "ina260.hpp"

namespace sensors
{
class Ina260CurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface,
                            public tap::communication::sensors::voltage::VoltageSensorInterface,
                            private modm::pt::Protothread
{
public:
    void update() override { run(); }
    float getCurrentMa() const override { return currentMa; }
    float getVoltageMv() const override { return busVoltageMv; }
    //float getPowerMw() const { return powerMw; }
    bool isConnected() const { return ina.checkConnected(); }

private:
    bool run()
    {
        PT_BEGIN();
        while (true)
        {
            if (!ina.checkConnected())
            {
                PT_CALL(ina.initialize());  // retries every pass until the chip answers
            }
            else
            {
                PT_CALL(ina.readAll());
                currentMa = ina.currentMa();
                busVoltageMv = ina.busVoltageMv();
                //powerMw = ina.powerMw();
            }
            PT_YIELD();
        }
        PT_END();
    }

    Ina260<modm::platform::I2cMaster2> ina;
    float currentMa = 0, busVoltageMv = 0;
    //powerMw = 0;
};
}  // namespace sensors