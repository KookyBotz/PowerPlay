package org.firstinspires.ftc.teamcode.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftProfiledCommand extends InstantCommand {
    public LiftProfiledCommand(LiftSubsystem lift, LiftSubsystem.LiftState state) {
        super(
                () -> lift.update(state)
        );
    }
}
