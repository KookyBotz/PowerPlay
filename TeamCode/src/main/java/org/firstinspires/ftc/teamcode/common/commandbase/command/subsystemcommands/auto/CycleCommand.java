package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(Robot robot) {
        super(
                //extend intake to pick up
                // TODO replace with TBD IntakeCommand
                new InstantCommand(() -> robot.intake.setDVA(350, 450, 2500)),
                new InstantCommand(() -> robot.intake.resetTimer()),
                new InstantCommand(() -> robot.intake.openClaw()),
                new InstantCommand(() -> robot.intake.extendForebar()),
                new InstantCommand(() -> robot.intake.intakeTurret()),

                //extend slides to deposit
                // TODO replace with new LiftCommand
                new InstantCommand(() -> robot.lift.setDVA(610, 400, 3750)),
                new InstantCommand(() -> robot.lift.resetTimer()),

                //wait until ready to intake
                new WaitUntilCommand(() -> robot.intake.getPos() > 340 && robot.lift.getPos() > 585),
                new WaitCommand(500),

                // deposit
                // TODO replace with new LiftCommand
                new InstantCommand(() -> robot.lift.setDVA(-610, -750, -7500)),
                new InstantCommand(() -> robot.lift.resetTimer()),
                //intake
                new InstantCommand(() -> robot.intake.closeClaw()),
                new WaitCommand(200),
                new InstantCommand(() -> robot.intake.transitionFourbar()),
                new InstantCommand(() -> robot.intake.depositTurret()),
                // TODO replace with TBD IntakeCommand
                new InstantCommand(() -> robot.intake.setDVA(-350, -450, -2500)),
                new InstantCommand(() -> robot.intake.resetTimer()),

                //transfer
                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                new InstantCommand(() -> robot.intake.closeForebar()),
                new WaitUntilCommand(() -> robot.intake.getPos() < 10),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.openClaw()),
                new WaitCommand(750)
        );
    }
}
