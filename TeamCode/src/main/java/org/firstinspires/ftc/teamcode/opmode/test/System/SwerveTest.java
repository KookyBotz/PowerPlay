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
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitController;

@TeleOp
@Config
public class SwerveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Localizer localizer = new TwoWheelLocalizer(
                () -> robot.horizontalEncoder.getPosition(),
                () -> robot.lateralEncoder.getPosition(),
                robot::getAngle
        );
        robot.localizer = localizer;

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();


        waitForStart();
        robot.startIMUThread(this);

        long time = System.currentTimeMillis();

        while (opModeIsActive()) {
            robot.read();
            Pose drive = new Pose(
                    new Point(Math.pow(gamepad1.left_stick_y, 3),
                            Math.pow(-gamepad1.left_stick_x, 3)).rotate(0),
                    Math.pow(-gamepad1.right_stick_x, 3)
            );

            //-robot.getAngle()
            robot.drivetrain.set(drive);
            robot.drivetrain.updateModules();

            long currTime = System.currentTimeMillis();
            telemetry.addData("hz", 1000 / (currTime - time));
//            telemetry.addData("target", robot.drivetrain.leftFrontModule.getTargetRotation());
//            telemetry.addData("current", robot.drivetrain.leftFrontModule.getModuleRotation());

            telemetry.addData("position:", robot.localizer.getPos());
            telemetry.addLine(robot.drivetrain.getTelemetry());
            time = currTime;

            telemetry.update();
            localizer.periodic();
            robot.write();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }

    }
}
