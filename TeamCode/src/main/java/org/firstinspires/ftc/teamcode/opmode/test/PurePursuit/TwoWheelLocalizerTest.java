package org.firstinspires.ftc.teamcode.opmode.test.PurePursuit;

import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.freightfrenzy.MecanumRobot;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitController;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.function.DoubleSupplier;

@TeleOp
@Config
public class TwoWheelLocalizerTest extends LinearOpMode {

    public static double coordX = 20;
    public static double coordY = 20;
    public static double coordHeading = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot(hardwareMap);
        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new TwoWheelLocalizer(horizontalPos, lateralPos, imuAngle);

        Pose targetPose = new Pose(coordX, coordY, coordHeading);
        waitForStart();

        long time = System.currentTimeMillis();

        while (opModeIsActive()) {
            targetPose.x = coordX;
            targetPose.y = coordY;
            targetPose.heading = coordHeading;
            localizer.periodic();
//            robot.drivetrain.set(
//                    new Pose(
//                            -gamepad1.left_stick_y,
//                            gamepad1.left_stick_x,
//                            gamepad1.right_stick_x
//                    )
//            );

            Pose powers = PurePursuitController.goToPosition(
                    localizer.getPos(), targetPose, new Pose(pCoefficientX, pCoefficientY, pCoefficientH)
            );

            telemetry.addData("powers", powers);

            robot.drivetrain.set(powers);

            long currTime = System.currentTimeMillis();
            telemetry.addData("hz", 1000 / (currTime - time));
            time = currTime;

            telemetry.addData("pose", localizer.getPos());
            telemetry.update();
        }

    }
}
