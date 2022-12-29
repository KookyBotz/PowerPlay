package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;

public class SwerveXCommand extends CommandBase {
    private SwerveDrivetrain drivetrain;
    private ElapsedTime timeout;

    public SwerveXCommand(SwerveDrivetrain swerveDrivetrain) {
        drivetrain = swerveDrivetrain;
    }

    @Override
    public void execute() {
        if (timeout == null) timeout = new ElapsedTime();
        drivetrain.leftFrontModule.setTargetRotation(-PI / 4);
        drivetrain.rightFrontModule.setTargetRotation(PI / 4);
        drivetrain.rightRearModule.setTargetRotation(-PI / 4);
        drivetrain.leftRearModule.setTargetRotation(PI / 4);
        drivetrain.updateModules();
    }

    @Override
    public boolean isFinished() {
        return timeout.seconds() > 9;
    }
}
