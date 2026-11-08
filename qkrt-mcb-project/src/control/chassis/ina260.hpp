#pragma once
#include <cstdint>
#include <modm/architecture/interface/i2c_device.hpp>
#include <modm/processing/resumable.hpp>

namespace sensors 
{
template < class I2cMaster >
class Ina260 : public modm::I2cDevice<I2cMaster, 4>
{
    enum class Register : uint8_t
    {
        CURRENT = 0x1,
        BUS_VOLTAGE = 0x2,
        //POWER = 0x3,
        MANUFACTURER_ID = 0xFE, 
    };

public: 
    static constexpr uint8_t DEFAULT_ADDRESS = 0x40; // A0 = A1 = GND
    static constexpr uint16_t MANUFACTURER_ID = 0x5449;

    //explicit to prevent unintended automatic conversions
    explicit Ina260(uint8_t address = DEFAULT_ADDRESS) 
        : modm::I2cDevice<I2cMaster, 4>(address) {}

    modm::ResumableResult<bool> 
    initialize() 
    {
        RF_BEGIN();
        ok = RF_CALL(readRegister(Register::MANUFACTURER_ID, rawID));
        connected = ok && (rawID == MANUFACTURER_ID);
        RF_END_RETURN(connected);
    }

    modm::ResumableResult<bool> 
    readAll()
    {
        RF_BEGIN();
        ok = RF_CALL(readRegister(Register::CURRENT, rawCurrent));
        if (!ok) { RF_RETURN(false); }
        ok = RF_CALL(readRegister(Register::BUS_VOLTAGE, rawVoltage));
        //if (!ok) { RF_RETURN(false); }
        //ok = RF_CALL(readRegister(Register::POWER, rawPower));
        RF_END_RETURN(ok);    
    }

    bool checkConnected() const { return connected; }
    float currentMa() const { return static_cast<int16_t>(rawCurrent) * 1.25f; } //1 bit = 1.25 mA
    float busVoltageMv() const { return rawVoltage * 1.25f; } //1 bit = 1.25 mV
    //float powerMw() const { return rawPower * 10.0f; } //1 bit = 10 mW

private:
    modm::ResumableResult<bool>
    readRegister(Register reg, uint16_t &value)
    {
        RF_BEGIN();
        txBuf[0] = static_cast<uint8_t>(reg);
        this->transaction.configureWriteRead(txBuf, 1, rxBuf, 2);
        txOk = RF_CALL(this->runTransaction());
        if (txOk) { value = (static_cast<uint16_t>(rxBuf[0]) << 8 | rxBuf[1]); }
        RF_END_RETURN(txOk);
    }

    uint8_t txBuf[1];
    uint8_t rxBuf[2];
    uint16_t rawID = 0, rawCurrent = 0, rawVoltage = 0;
    //rawPower = 0;
    bool ok = false, txOk = false, connected = false;    
};
} //namespace sensors