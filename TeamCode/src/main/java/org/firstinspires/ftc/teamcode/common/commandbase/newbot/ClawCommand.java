package org.firstinspires.ftc.teamcode.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class ClawCommand extends InstantCommand {
    public ClawCommand(IntakeSubsystem intake, IntakeSubsystem.ClawState state) {
        super(
                () -> intake.update(state)
        );
    }
}
