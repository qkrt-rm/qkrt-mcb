#include "logger.hpp"

namespace communication::serial
{

Logger::Logger(tap::Drivers* drivers)
    : _M_timeout()
    , _M_device(drivers)
    , _M_stream(_M_device)
{
}

}  // namespace communication::serial