package org.firstinspires.ftc.teamcode.opmode.test.System;

import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitController;

@TeleOp
@Config
public class SwerveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
        waitForStart();
        robot.startIMUThread(this);

        long time = System.currentTimeMillis();

        while (opModeIsActive()) {
            Pose drive = new Pose(
                    new Point(gamepad1.left_stick_y,
                            -gamepad1.left_stick_x).rotate(0),
                    -gamepad1.right_stick_x
            );
            robot.drivetrain.set(drive);
            robot.drivetrain.updateModules();

            long currTime = System.currentTimeMillis();
            telemetry.addData("hz", 1000 / (currTime - time));
            time = currTime;

            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }

    }
}
