package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private int pos;

    public IntakeCommand(IntakeSubsystem intake, int pos) {
        this.intake = intake;
        this.pos = pos;
    }

    @Override
    public void execute() {
        intake.setExtension(pos);
    }
}
