package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(Robot robot) {
        super (
                new InstantCommand(() -> robot.intake.newProfile(300, 400, 2500)),
                new InstantCommand(() -> robot.intake.resetTimer()),
                new InstantCommand(() -> robot.intake.openClaw()),
                new InstantCommand(() -> robot.intake.extendFourbar()),
                new InstantCommand(() -> robot.intake.intakeTurret()),

                new WaitUntilCommand(() -> robot.intake.getPos() > 340 && robot.lift.getPos() > 580),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.closeClaw()),

                new WaitCommand(200),
                new InstantCommand(() -> robot.intake.transitionFourbar()),
                new InstantCommand(() -> robot.intake.depositTurret()),

                new InstantCommand(() -> robot.intake.newProfile(0, 300, 2500)),
                new InstantCommand(() -> robot.intake.resetTimer()),

                new InstantCommand(() -> robot.intake.closeFourbar()),
                new WaitUntilCommand(() -> robot.intake.getPos() < 10),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.openClaw()),
                new WaitCommand(750)
        );

    }
}
