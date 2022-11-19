package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class LatchCommand extends InstantCommand {
    public LatchCommand(Robot robot, LiftSubsystem.LatchState state) {
        super(
                () -> robot.lift.update(state)
        );
    }
}
