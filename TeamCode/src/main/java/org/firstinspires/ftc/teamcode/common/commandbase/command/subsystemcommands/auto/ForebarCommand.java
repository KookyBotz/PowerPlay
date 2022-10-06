package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class ForebarCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double pos;

    public ForebarCommand(IntakeSubsystem intake, double pos) {
        this.intake = intake;
        this.pos = pos;
    }

    @Override
    public void execute() { intake.setForebar(pos);}
}
