package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(Robot robot) {
        super(
            new InstantCommand(() -> robot.intake.closeClaw()),
            new WaitCommand(500),
            new InstantCommand(() -> robot.intake.newProfile(-5, 750, 2500)),
            new WaitUntilCommand(() -> robot.intake.getPos() < 10),
            new WaitUntilCommand(() -> robot.lift.getPos() < 10),
            new InstantCommand(() -> robot.intake.depositTurret()),
            new InstantCommand(() -> robot.intake.closeForebar())
        );
    }
}
