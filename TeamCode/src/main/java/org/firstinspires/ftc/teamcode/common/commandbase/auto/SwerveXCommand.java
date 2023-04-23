package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;

import java.util.function.BooleanSupplier;

public class SwerveXCommand extends CommandBase {
    private SwerveDrivetrain drivetrain;
    private ElapsedTime timeout;
    private BooleanSupplier retargeting;
    private double angle;

    public SwerveXCommand(SwerveDrivetrain swerveDrivetrain, BooleanSupplier retargeting, double angle) {
        drivetrain = swerveDrivetrain;
        this.retargeting = retargeting;
        this.angle = angle;
    }

    public SwerveXCommand(SwerveDrivetrain swerveDrivetrain){
        this(swerveDrivetrain, ()->false, 0.23);
    }

    @Override
    public void execute() {
        if (!retargeting.getAsBoolean()) {
            if (timeout == null) timeout = new ElapsedTime();
            drivetrain.frontLeftModule.setTargetRotation(angle);
            drivetrain.frontRightModule.setTargetRotation(angle);
            drivetrain.backRightModule.setTargetRotation(angle);
            drivetrain.backLeftModule.setTargetRotation(angle);
            drivetrain.updateModules();
        }
    }

    @Override
    public boolean isFinished() {
        return timeout.seconds() > 7;
    }
}
