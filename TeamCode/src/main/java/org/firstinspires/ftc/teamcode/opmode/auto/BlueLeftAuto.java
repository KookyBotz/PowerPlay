package org.firstinspires.ftc.teamcode.opmode.auto;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPathBuilder;

@Autonomous (name = "BlueLeftAuto")
public class BlueLeftAuto extends LinearOpMode {

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
                .setPower(1.0)
                .setMotionProfile(new MotionProfile(0.5, 1))
                .then(new Pose(-5, -32, 0))
                .then(new Pose(-77, -32, 0))
                .setFollowDistance(0)
                .then(new Pose(-77, -15, -Math.PI / 5))
                .then(new Pose(-77, -23, 0))
                .then(new Pose(-77, -15, -Math.PI / 5))
                .then(new Pose(-77, -23, 0))
                .then(new Pose(-77, -15, -Math.PI / 5))
                .then(new Pose(-77, -23, 0))
                .then(new Pose(-77, -15, -Math.PI / 5))
                .then(new Pose(-77, -23, 0))
                .then(new Pose(-77, -15, -Math.PI / 5))
                .then(new Pose(-77, -23, 0))
                .setFollowDistance(10)
                .then(new Pose(-77, -32, 0))
                .then(new Pose(-33, -32, 0))
                // stay still if left
                // go somewhere else if other case

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
