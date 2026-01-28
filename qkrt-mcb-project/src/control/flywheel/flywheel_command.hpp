#pragma once

#include "tap/control/command.hpp"

#include "flywheel_subsystem.hpp"

namespace control::flywheel
{
    class FlywheelOnCommand : public tap::control::Command
    {
    public:
        FlywheelOnCommand(FlywheelSubsystem &flywheel, float flywheel_speed);

        void initialize() override;

        void execute() override;

        void end(bool interuppted) override;

        bool isFinished() const override;

        const char *getName() const override { return "flywheel on command";}
    private:
        FlywheelSubsystem &m_flywheel;

        float m_flywheelSpeed;
        static constexpr float OFF_SPEED = 0;

    };
} // namespace control::flywheel
