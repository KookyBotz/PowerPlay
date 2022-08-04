package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

@TeleOp
@Config
public class LocalizerTest extends LinearOpMode {
    public static int pCoefficientX = 26;
    public static int pCoefficientY = 24;
    public static double pCoefficientH = Math.PI / 2.5;

    public static double coordX = 20;
    public static double coordY = 20;
    public static double coordHeading = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotPP robot = new RobotPP(hardwareMap);
        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new TwoWheelOdo(horizontalPos, lateralPos, imuAngle);

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
