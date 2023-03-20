package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    Robot robot;
    double loopTime = 0;
    ElapsedTime timer;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, false);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void loop() {
        if (timer == null) {
            timer = new ElapsedTime();
            timer.reset();
        }
//        if (gamepad1.a) {
//            robot.drivetrain.leftFrontModule.setMotorPower(1);
//        } else {
//            robot.drivetrain.leftFrontModule.setMotorPower(0);
//        }
//
//        if (gamepad1.b) {
//            robot.drivetrain.rightFrontModule.setMotorPower(1);
//        } else {
//            robot.drivetrain.rightFrontModule.setMotorPower(0);
//        }
//
//        if (gamepad1.x) {
//            robot.drivetrain.leftRearModule.setMotorPower(1);
//        } else {
//            robot.drivetrain.leftRearModule.setMotorPower(0);
//        }
//
//        if (gamepad1.y) {
//            robot.drivetrain.rightRearModule.setMotorPower(1);
//        } else {
//            robot.drivetrain.rightRearModule.setMotorPower(0);
//        }

        robot.read();

        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        // Drivetrain
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3)),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3))).rotate(robot.getAngle() - SwerveDrivetrain.imuOffset),
                (Math.pow(-gamepad1.right_stick_x, 3))
        );

        if (timer.seconds() > 5) {
            gamepad1.rumble(500);
            timer.reset();
        }
//
//        if (gamepad1.left_bumper) {
//            SwerveDrivetrain.imuOff = robot.getAngle() + Math.PI;
//        }
//        for (SwerveModule module : robot.drivetrain.modules) {
//            module.setTargetRotation(0);
////            telemetry.addData(" module.getModuleRotation())
//        }
//        telemetry.addData("leftRear", robot.drivetrain.leftRearModule.getTelemetry("leftRear"));
//        telemetry.addData("leftFront", robot.drivetrain./**/leftFrontModule.getTelemetry("leftFront"));
//        telemetry.addData("rightRear", robot.drivetrain.rightRearModule.getTelemetry("rightRear"));
//        telemetry.addData("rightFront", robot.drivetrain.rightFrontModule.getTelemetry("rightFront"));

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));

        loopTime = loop;
        telemetry.update();
        robot.drivetrain.set(drive);

        robot.drivetrain.updateModules();

        robot.write();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
}
