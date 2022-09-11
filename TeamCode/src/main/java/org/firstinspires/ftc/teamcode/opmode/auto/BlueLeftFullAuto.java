package org.firstinspires.ftc.teamcode.opmode.auto;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPathBuilder;

@Autonomous (name = "BlueLeftFullAuto")
public class BlueLeftFullAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Localizer localizer = new BetterSwerveLocalizer(() -> -robot.getAngle(), robot.drivetrain.modules);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        while (!isStarted()) {
            for (SwerveModule module : robot.drivetrain.modules) {
                module.setTargetRotation(0);
            }
            robot.drivetrain.updateModules();
            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

        waitForStart();
        robot.startIMUThread(this);

        PurePursuitPath path = new PurePursuitPathBuilder()
                .setDrivetrain(drivetrain)
                .setLocalizer(localizer)
                .setController(true)
                .setFollowDistance(15)
                .setPower(0.6)
                .setMotionProfile(new MotionProfile(0.5, 1))
                .then(new Pose(6, 90, Math.PI))
                .then(new Pose(24, 84, Math.PI))
                .then(new Pose(60, 84, Math.PI))
                .then(new Pose(60, 120, 7 * Math.PI / 6))
                .then(new Pose(60, 132, Math.PI))
                .then(new Pose(60, 120, 7 * Math.PI / 6))
                .then(new Pose(60, 132, Math.PI))
                .then(new Pose(60, 120, 7 * Math.PI / 6))
                .then(new Pose(60, 132, Math.PI))
                .then(new Pose(60, 120, 7 * Math.PI / 6))
                .then(new Pose(60, 132, Math.PI))
                .then(new Pose(60, 120, 7 * Math.PI / 6))
                .then(new Pose(60, 132, Math.PI))
                .then(new Pose(60, 120, 7 * Math.PI / 6))
                // cycle other side
                .then(new Pose(60, 12, 0))
                .then(new Pose(60, 24, Math.PI / 6))
                .then(new Pose(60, 12, 0))
                .then(new Pose(60, 24, Math.PI / 6))
                .then(new Pose(60, 12, 0))
                .then(new Pose(60, 24, Math.PI / 6))
                .then(new Pose(60, 12, 0))
                .then(new Pose(60, 24, Math.PI / 6))
                .then(new Pose(60, 12, 0))
                .then(new Pose(60, 24, Math.PI / 6))
                .then(new Pose(60, 132, 0))
                .then(new Pose())

                .build();

        while (opModeIsActive()) {
            localizer.periodic();
            path.update();
            robot.drivetrain.updateModules();

            telemetry.addData("current pose", localizer.getPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
