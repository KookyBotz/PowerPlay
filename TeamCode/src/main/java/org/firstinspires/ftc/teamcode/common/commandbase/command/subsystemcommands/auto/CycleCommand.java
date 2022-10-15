package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(Robot robot) {
        super (
            new ParallelCommandGroup(
                new InstantCommand(() -> robot.intake.setPos(400)),
                new InstantCommand(() -> robot.lift.setPos(500)),
                new InstantCommand(() -> robot.lift.resetTimer())
            ),
            new WaitCommand(3000),
            new ParallelCommandGroup(
                    new InstantCommand(() -> robot.intake.setPos(-400)),
                    new InstantCommand(() -> robot.intake.resetTimer()),
                    new InstantCommand(() -> robot.lift.setPos(-500)),
                    new InstantCommand(() -> robot.lift.resetTimer())
            )
        );
    }
}
