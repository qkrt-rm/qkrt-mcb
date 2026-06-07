#pragma once

#include "tap/control/command.hpp"
#include "control/control_operator_interface.hpp"
#include "drivers.hpp"
#include "m3508_velocity_agitator_subsystem.hpp"

namespace control::agitator::m3508
{
    class M3508AgitatorCommand : public tap::control::Command
    {
    public:

        M3508AgitatorCommand(Drivers& drivers, M3508AgitatorSubsystem& agitator, 
                          float speed, tap::control::Command* flywheelsCommand);

        bool isReady() override;
        void initialize() override {};
        void execute() override;
        void end(bool interuppted) override;
        bool isFinished() const override;
        const char *getName() const override { return "agitator on command";}

    private:

        Drivers* m_drivers;
        M3508AgitatorSubsystem& m_agitator;
        tap::control::Command* m_flywheelsCommand;

        float m_agitatorSpeed;
        static constexpr float OFF_SPEED = 0.0f;
    };
} // namespace control::agitator::m3508