#pragma once

#include "tap/control/command.hpp"
#include "control/control_operator_interface.hpp"

#include "m3508_flywheel_subsystem.hpp"

namespace control::flywheel::m3508
{
    class M3508FlywheelOnCommand : public tap::control::Command
    {
    public:
        M3508FlywheelOnCommand(M3508FlywheelSubsystem& flywheel, 
                          float speed);

        void initialize() override;

        void execute() override;

        void end(bool interuppted) override;

        bool isFinished() const override {return false;}

        const char *getName() const override { return "flywheel on command";}
    private:
        M3508FlywheelSubsystem& m_flywheel;

        float m_flywheelSpeed;
        static constexpr float OFF_SPEED = 0;
    };
} // namespace control::flywheel