package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class ClearFourbarCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double pos;

    public ClearFourbarCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        if (intake.getFourbarPos() >= intake.fourbar_transition) {
            intake.update(IntakeSubsystem.FourbarState.TRANSITION);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
