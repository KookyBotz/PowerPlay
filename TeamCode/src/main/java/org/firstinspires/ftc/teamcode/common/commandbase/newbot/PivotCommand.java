package org.firstinspires.ftc.teamcode.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class PivotCommand extends InstantCommand {
    public PivotCommand(IntakeSubsystem intake, IntakeSubsystem.PivotState state) {
        super(
                () -> intake.update(state)
        );
    }
}
