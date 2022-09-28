package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class ExtensionCommand extends CommandBase {
    private IntakeSubsystem intake;
    private int pos;

    public ExtensionCommand(IntakeSubsystem intake, int pos) {
        this.intake = intake;
        this.pos = pos;
    }

    @Override
    public void execute() {
        intake.setExtension(pos);
    }
}
