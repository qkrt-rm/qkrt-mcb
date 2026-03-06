#pragma once

#include "drivers.hpp" 
#include <tap/control/subsystem.hpp>

namespace control::flywheel {

    class FlywheelSubsystem : public tap::control::Subsystem {
    public:
        FlywheelSubsystem(Drivers& drivers) : tap::control::Subsystem(&drivers) {}
        
        virtual ~FlywheelSubsystem() = default;
        
        // standard methods
        virtual void setTargetSpeed(float speed) = 0; 
        
        const char* getName() const { return "Flywheel"; } 
    };

} // namespace control::flywheel