package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class CycleCommand extends ParallelCommandGroup {
    public CycleCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.intake.setDVA(400, 750, 2500)),
                new InstantCommand(() -> robot.intake.resetTimer()),
                new InstantCommand(() -> robot.intake.openClaw()),
                new InstantCommand(() -> robot.intake.extendForebar()),
                new InstantCommand(() -> robot.intake.intakeTurret()),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robot.intake.getPos() > 380),
                        new WaitCommand(200),
                        new InstantCommand(() -> robot.intake.closeClaw()),
                        new WaitCommand(200),
                        new InstantCommand(() -> robot.intake.closeForebar()),
                        new InstantCommand(() -> robot.intake.depositTurret()),
                        new InstantCommand(() -> robot.intake.setDVA(-400, -750, -2500)),
                        new InstantCommand(() -> robot.intake.resetTimer())
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.lift.setDVA(525, 1000, 7500)),
                        new InstantCommand(() -> robot.lift.resetTimer()),
                        new WaitUntilCommand(()->robot.lift.getPos() > 500),
                        new InstantCommand(() -> robot.lift.setDVA(-525, -1000, -7500)),
                        new InstantCommand(() -> robot.lift.resetTimer())
                ),
                new WaitUntilCommand(() -> robot.intake.getPos() < 40 && robot.lift.getPos() < 40),
                new InstantCommand(() -> robot.intake.openClaw()).andThen(new WaitCommand(1000))
        );
    }
}
