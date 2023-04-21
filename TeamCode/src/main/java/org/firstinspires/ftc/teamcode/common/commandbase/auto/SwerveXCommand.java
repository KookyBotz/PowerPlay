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

    public SwerveXCommand(SwerveDrivetrain swerveDrivetrain, BooleanSupplier retargeting) {
        drivetrain = swerveDrivetrain;
        this.retargeting = retargeting;
    }

    public SwerveXCommand(SwerveDrivetrain swerveDrivetrain){
        this(swerveDrivetrain, ()->false);
    }

    @Override
    public void execute() {
        if (!retargeting.getAsBoolean()) {
            if (timeout == null) timeout = new ElapsedTime();
            drivetrain.frontLeftModule.setTargetRotation(-PI / 4);
            drivetrain.frontRightModule.setTargetRotation(PI / 4);
            drivetrain.backRightModule.setTargetRotation(-PI / 4);
            drivetrain.backLeftModule.setTargetRotation(PI / 4);
            drivetrain.updateModules();
        }
    }

    @Override
    public boolean isFinished() {
        return timeout.seconds() > 7;
    }
}
