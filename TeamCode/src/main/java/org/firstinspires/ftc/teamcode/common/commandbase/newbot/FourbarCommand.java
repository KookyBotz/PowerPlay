package org.firstinspires.ftc.teamcode.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class FourbarCommand extends InstantCommand {
    public FourbarCommand(IntakeSubsystem intake, IntakeSubsystem.FourbarState state) {
        super(
                () -> intake.update(state)
        );
    }
}
