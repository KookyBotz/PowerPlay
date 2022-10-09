package org.firstinspires.ftc.teamcode.opmode.test.PurePursuit;

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

import org.firstinspires.ftc.teamcode.common.freightfrenzy.SwerveRobot;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitController;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

@TeleOp
@Config
public class SwerveGoToPositionTest extends LinearOpMode {

    public static double coordX = 0;
    public static double coordY = 0;
    public static double coordHeading = Math.PI / 4;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveRobot robot = new SwerveRobot(hardwareMap);
//        DoubleSupplier horizontalPos = () -> robot.horizontalEncoder.getCurrentPosition(),
//                lateralPos = () -> robot.lateralEncoder.getCurrentPosition(),
//                imuAngle = () -> -robot.imu.getAngularOrientation().firstAngle;

        Localizer localizer = new BetterSwerveLocalizer(()->-robot.getAngle(), robot.drivetrain.modules);
        Pose targetPose = new Pose(coordX, coordY, coordHeading);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
        waitForStart();
        robot.startIMUThread(this);

        long time = System.currentTimeMillis();

        while (opModeIsActive()) {
            targetPose.x = coordX;
            targetPose.y = coordY;
            targetPose.heading = coordHeading;
            localizer.periodic();
            Pose drive;
            if(!gamepad1.a) {
                drive = new Pose(
                        new Point(-gamepad1.left_stick_y,
                                gamepad1.left_stick_x).rotate(-robot.getAngle()),
                        gamepad1.right_stick_x
                );
            }else{
                drive = PurePursuitController.goToPosition(
                        localizer.getPos(), targetPose, new Pose(pCoefficientX, pCoefficientY, pCoefficientH)
                );
            }
            robot.drivetrain.set(drive);
            robot.drivetrain.updateModules();

//



//            long currTime = System.currentTimeMillis();
//            telemetry.addData("hz", 1000 / (currTime - time));
//            time = currTime;

            telemetry.addData("pose", localizer.getPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
 
    }
}
