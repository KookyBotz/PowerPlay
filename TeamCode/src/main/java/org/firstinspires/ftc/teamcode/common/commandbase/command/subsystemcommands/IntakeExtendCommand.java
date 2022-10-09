package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Kinematics;

public class IntakeExtendCommand extends SequentialCommandGroup {
    public IntakeExtendCommand(Robot robot, double x, double y) {
        super(
                double[] arr = Kinematics.forebar(x, y);
//                new WaitUntilCommand(() -> robot.lift.getPos() < robot.lift.high_pos - 50),
//                new InstantCommand(() -> robot.intake.extensionOut())
//                .alongWith(new InstantCommand(() -> robot.intake.openClaw()))
//                .alongWith(new InstantCommand(() -> robot.intake.extendForebar()))
                new IntakeCommand(robot.intake, robot.intake.intake_out_pos)
                .alongWith(new ForebarCommand(robot.intake, ))
        );
    }
}
