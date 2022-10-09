package org.firstinspires.ftc.teamcode.opmode.test.PurePursuit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.freightfrenzy.MecanumRobot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.function.DoubleSupplier;

@TeleOp
@Config
public class MecanumPurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot(hardwareMap);
        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new TwoWheelLocalizer(horizontalPos, lateralPos, imuAngle);

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
