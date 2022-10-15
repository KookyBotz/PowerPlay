package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(Robot robot) {
        super(
//                new InstantCommand(() -> robot.intake.closeForebar()),
//                new InstantCommand(() -> robot.intake.extensionIn()),
//                new WaitUntilCommand(() -> robot.intake.getExtension() < robot.intake.extension_out_pos - 10)
//                new IntakeCommand(robot.intake, robot.intake.intake_in_pos)
        );

    }
}
