package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;

public class WartimeCommand extends InstantCommand {
    public WartimeCommand(SwerveDrivetrain drivetrain, double angle) {
        super(
                () -> {
                    drivetrain.frontLeftModule.setTargetRotation(angle);
                    drivetrain.frontRightModule.setTargetRotation(angle);
                    drivetrain.backRightModule.setTargetRotation(angle);
                    drivetrain.backLeftModule.setTargetRotation(angle);
                    drivetrain.updateModules();
                }
        );
    }
}
