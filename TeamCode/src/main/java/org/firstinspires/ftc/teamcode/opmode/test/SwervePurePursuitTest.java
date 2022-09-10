package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.freightfrenzy.SwerveRobot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

@TeleOp
@Config
public class SwervePurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveRobot robot = new SwerveRobot(hardwareMap);
//        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
//                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
//                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new BetterSwerveLocalizer(() -> -robot.getAngle(), robot.drivetrain.modules);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();


        waitForStart();
        robot.startIMUThread(this);

        long time = System.currentTimeMillis();

        PurePursuitPath path = new PurePursuitPath(robot.drivetrain, localizer,
                new Waypoint(new Pose(0, 0, 0), 10, 0.5),
                new Waypoint(new Pose(0, 60, Math.PI/2.0), 10, 0.5),
                new Waypoint(new Pose(-40, 70, Math.PI/2.0), 10, 1.0),
                new Waypoint(new Pose(20, 50, Math.PI/2.0), 10, 0.3),
                new Waypoint(new Pose(0, 0, Math.PI/2.0), 10, 0.5));

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
