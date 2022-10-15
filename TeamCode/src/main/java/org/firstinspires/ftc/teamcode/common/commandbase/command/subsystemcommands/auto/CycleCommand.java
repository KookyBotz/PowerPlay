package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class CycleCommand extends ParallelCommandGroup {
    public CycleCommand(Robot robot) {
        super (
            new InstantCommand(() -> robot.intake.setPos(400)),
            new InstantCommand(() -> robot.intake.openClaw()),
            new InstantCommand(() -> robot.intake.extendForebar()),
            new SequentialCommandGroup(
                new WaitUntilCommand(()->robot.intake.getPos() > 390),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.closeClaw()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.closeForebar()),
                new InstantCommand(() -> robot.intake.setPos(-400))
            ),
            new SequentialCommandGroup(
                new InstantCommand(() -> robot.lift.setPos(500)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.lift.setPos(-500))
            )
        );
    }
}
