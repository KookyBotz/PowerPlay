package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class LiftRetractCommand extends SequentialCommandGroup {
    public LiftRetractCommand(Robot robot) {
        super (
                new LatchCommand(robot, LiftSubsystem.LatchState.UNLATCHED),
                new WaitCommand(200),
                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LiftState.RETRACTED))
        );
    }
}
