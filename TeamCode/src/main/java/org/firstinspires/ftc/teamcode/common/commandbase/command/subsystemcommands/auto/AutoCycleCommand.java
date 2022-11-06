package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(Robot robot, int distance, double fourbarPos) {
        super(
                new InstantCommand(() -> robot.intake.newProfile(distance, 600, 1500)),
                new InstantCommand(() -> robot.intake.resetTimer()),
                new InstantCommand(() -> robot.intake.openClaw()),
                new InstantCommand(() -> robot.intake.setFourbar(fourbarPos)),
                new InstantCommand(() -> robot.intake.intakeTurret()),

                new InstantCommand(() -> robot.lift.newProfile(610, 800, 2500)),

                new WaitUntilCommand(() -> robot.intake.getPos() > distance - 30 && robot.lift.getPos() > 580),
                new WaitCommand(250),

                new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
                //intake
                new InstantCommand(() -> robot.intake.closeClaw()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.transitionFourbar()),
                new WaitCommand(400),
                new InstantCommand(() -> robot.intake.depositTurret()),

                new InstantCommand(() -> robot.intake.newProfile(-5, 750, 2500)),

                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                new WaitUntilCommand(() -> robot.intake.getPos() < 10),
                new InstantCommand(() -> robot.intake.closeForebar()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.openClaw()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.transitionFourbar()),
                new WaitCommand(400)
        );
    }
}
