#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/communication/serial/uart_terminal_device.hpp>

namespace tap
{
class Drivers;
}  // namespace tap

namespace communication::serial
{

class Logger
{
public:
    explicit Logger(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(Logger);
    ~Logger() = default;

    void initialize()
    {
        _M_device.initialize();
    }

    template<typename... Args>
    void printf(const char* fmt, Args&&... args)
    {
        if (_M_timeout.execute())
            _M_stream.printf(fmt, std::forward<Args>(args)...);
        
    }

    void delay(uint32_t ms)
    {
        // TODO: keep track of last message that failed to send due to _M_timeout.execute()
        //       returning false. if the condition below holds true, check whether the
        //       timestamp of the failed message is "close enough" to the current time. if
        //       the message was attempted to be sent right before this delay, send it anyway
        if (_M_timeout.isStopped() || _M_timeout.isExpired())
            _M_timeout.restart(ms);
    }

private:
    tap::arch::MilliTimeout _M_timeout;
    tap::communication::serial::UartTerminalDevice _M_device;
    modm::IOStream _M_stream;
};

}  // namespace communication::serial