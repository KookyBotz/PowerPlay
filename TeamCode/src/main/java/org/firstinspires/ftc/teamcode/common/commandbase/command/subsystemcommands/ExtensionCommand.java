package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Kinematics;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.test.System.ExtensionTest;

public class ExtensionCommand extends SequentialCommandGroup {
    private IntakeSubsystem intake;
    private double[] points;

    public ExtensionCommand(Robot robot, double x, double y) {
        super(

            new IntakeCommand(robot.intake, (int) Kinematics.forebar(x, y, robot.intake.FOURBAR_LENGTH)[0])
        );
        this.intake = intake;
        this.points = Kinematics.forebar(x, y, intake.FOURBAR_LENGTH);


    }
}
