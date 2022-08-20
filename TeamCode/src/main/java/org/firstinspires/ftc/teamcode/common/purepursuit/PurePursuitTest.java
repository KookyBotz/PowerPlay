package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.TwoWheelOdo;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.function.DoubleSupplier;

@TeleOp
@Config
public class PurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumChassis robot = new MecanumChassis(hardwareMap);
        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new TwoWheelOdo(horizontalPos, lateralPos, imuAngle);

        waitForStart();

        long time = System.currentTimeMillis();

        PurePursuitPath path = new PurePursuitPath(robot.drivetrain, localizer,
                new Waypoint(new Pose(0, 0, 0), 10),
                new Waypoint(new Pose(40, 0, Math.toRadians(90)), 10),
                new Waypoint(new Pose(40, 40, 0), 10));

        while (opModeIsActive()) {
            localizer.periodic();

            path.update();

            long currTime = System.currentTimeMillis();
            telemetry.addData("hz", 1000 / (currTime - time));
            time = currTime;

            telemetry.addData("pose", localizer.getPos());
            telemetry.update();
        }

    }
}
