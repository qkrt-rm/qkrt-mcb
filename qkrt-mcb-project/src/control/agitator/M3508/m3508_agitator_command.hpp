#pragma once

#include "tap/control/command.hpp"
#include "control/control_operator_interface.hpp"

#include "m3508_velocity_agitator_subsystem.hpp"

namespace control::agitator::m3508
{
    class M3508AgitatorCommand : public tap::control::Command
    {
    public:
        M3508AgitatorCommand(M3508AgitatorSubsystem& agitator, 
                          float speed);

        void initialize() override;

        void execute() override;

        void end(bool interuppted) override;

        bool isFinished() const override {return false;}

        const char *getName() const override { return "agitator on command";}
    private:
        M3508AgitatorSubsystem& m_agitator;

        float m_agitatorSpeed;
        static constexpr float OFF_SPEED = 0;
    };
} // namespace control::agitator::m3508