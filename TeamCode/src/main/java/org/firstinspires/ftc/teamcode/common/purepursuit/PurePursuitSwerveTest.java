package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.TwoWheelOdo;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.function.DoubleSupplier;

@TeleOp
@Config
public class PurePursuitSwerveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveChassis robot = new SwerveChassis(hardwareMap);
//        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
//                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
//                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new BetterSwerveLocalizer(robot::getAngle, robot.drivetrain.modules);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
        waitForStart();
        robot.startIMUThread(this);

        long time = System.currentTimeMillis();

        PurePursuitPath path = new PurePursuitPath(robot.drivetrain, localizer,
                new Waypoint(new Pose(0, 0, 0), 10),
                new Waypoint(new Pose(0, 40, 0), 10),
                new Waypoint(new Pose(40, 40, 0), 10),
                new Waypoint(new Pose(40, 0, 0), 10),
                new Waypoint(new Pose(0, 0, 0), 10));

        while (opModeIsActive()) {
            localizer.periodic();
            path.update();
            robot.drivetrain.updateModules();


//            long currTime = System.currentTimeMillis();
//            telemetry.addData("hz", 1000 / (currTime - time));
//            time = currTime;

            telemetry.addData("pose", localizer.getPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
