package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public class IntakeExtendCommand extends SequentialCommandGroup {
    public IntakeExtendCommand(Robot robot) {
        super(
//                new WaitUntilCommand(() -> robot.lift.getPos() < robot.lift.high_pos - 50),
//                new InstantCommand(() -> robot.intake.extensionOut())
//                .alongWith(new InstantCommand(() -> robot.intake.openClaw()))
//                .alongWith(new InstantCommand(() -> robot.intake.extendForebar()))
                new IntakeCommand(robot.intake, robot.intake.intake_out_pos)
        );
    }
}
