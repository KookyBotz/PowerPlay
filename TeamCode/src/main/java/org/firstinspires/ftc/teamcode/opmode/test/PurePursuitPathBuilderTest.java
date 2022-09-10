package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.freightfrenzy.SwerveRobot;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPathBuilder;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

@TeleOp
@Config
public class PurePursuitPathBuilderTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SwerveRobot swerve = new SwerveRobot(hardwareMap);
        Drivetrain drivetrain = swerve.drivetrain;
        Localizer localizer = new BetterSwerveLocalizer(() -> -swerve.getAngle(), swerve.drivetrain.modules);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();



        while (!isStarted()) {
            System.out.println(isStarted());
            for (SwerveModule module : swerve.drivetrain.modules) {
                module.setTargetRotation(0);
            }
            swerve.drivetrain.updateModules();
            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

        waitForStart();
        swerve.startIMUThread(this);

        // higher speeds = higher follow distances
        PurePursuitPath path = new PurePursuitPathBuilder()
                .setDrivetrain(drivetrain)
                .setLocalizer(localizer)
                .setController(true)
                .setFollowDistance(15)
                .setPower(1.0)
                .setMotionProfile(new MotionProfile(0.25, 1))
                .then(new Pose())
                .then(new Pose(0, 60, 0))
                .then(new Pose(60, 60, 0))
                .then(new Pose(60, 0, 0))
                .then(new Pose())
                .build();

        while (opModeIsActive()) {
            localizer.periodic();
            path.update();
            swerve.drivetrain.updateModules();

            telemetry.addData("pose", localizer.getPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
