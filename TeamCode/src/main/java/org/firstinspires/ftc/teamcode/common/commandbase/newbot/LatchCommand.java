package org.firstinspires.ftc.teamcode.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LatchCommand extends InstantCommand {
    public LatchCommand(LiftSubsystem lift, LiftSubsystem.LatchState state) {
        super(
                () -> lift.update(state)
        );
    }
}
