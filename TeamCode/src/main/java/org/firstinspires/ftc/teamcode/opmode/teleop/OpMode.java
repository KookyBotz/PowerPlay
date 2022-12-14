package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Math.PI;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.FileInterface;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

import java.io.File;
import java.io.IOException;

@Config
@TeleOp(name = "OpMode👌👌😍🎶🎶😎")
public class OpMode extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;

    private boolean xLock = false;
    private boolean pDRS = false;

    private boolean pDLB = false;
    private boolean pDRB = false;
    private boolean pDLT = false;
    private boolean pDRT = false;

    private boolean pDBA = false;
    private boolean pDBX = false;
    private boolean pDBY = false;
    private boolean pDBB = false;

    private boolean pCone = false;

    private boolean busy = false;
//    private boolean busyCone = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/data.json");
        try {
            file.createNewFile();

        } catch (IOException e) {
            System.out.println("amogus squared");
        }
        FileInterface.write(FileInterface.IMU, "169.3");
        FileInterface.write(FileInterface.LIFT, "15");
        FileInterface.write(FileInterface.INTAKE, "amogus");

        robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.intake.setFourbar(robot.intake.fourbar_transition);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();
        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        // Drivetrain
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3) * speedMultiplier),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3)) * speedMultiplier).rotate(robot.getAngle() - SwerveDrivetrain.imuOff),
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier
        );

        if (gamepad1.left_bumper) {
            SwerveDrivetrain.imuOff = robot.getAngle();
            robot.intake.extension.resetEncoder();
            robot.lift.lift.resetEncoder();
        }

        // Gamepad2
        if (gamepad2.dpad_up) {
            robot.lift.setSlideFactor(1);
        } else if (gamepad2.dpad_down) {
            robot.lift.setSlideFactor(-1);
        }

        if (gamepad2.dpad_left && !busy) {
            busy = true;
            CommandScheduler.getInstance().schedule(new TeleopCycleCommand(robot, (b) -> busy = b));
        }

        boolean dLT = (gamepad2.left_trigger > 0.3);
        boolean dRT = (gamepad2.right_trigger > 0.3);
//        boolean hasCone = robot.intake.hasCone();
        if (dLT && !pDLT) {
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
        } else if (dRT && !pDRT) {
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)));
        }
            //        } else if (hasCone && !pCone && !busyCone) {
////            busyCone = true;
////            schedule(new SequentialCommandGroup(
////                    new WaitCommand(200),
////                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)))
////            );
//        }
//        pCone = hasCone;
        pDLT = dLT;
        pDRT = dRT;

        double gamepad2_left_stick_y = gamepad2.left_stick_y;
        if (Math.abs(gamepad2_left_stick_y) > 0.1) {
            robot.intake.setFourbarFactor(Math.pow(gamepad2_left_stick_y, 3));
        }

        double gamepad2_left_stick_x = gamepad2.left_stick_x;
        if (Math.abs(gamepad2_left_stick_x) > 0.01) {
            robot.intake.setSlideFactor(Math.pow(-gamepad2_left_stick_x, 3));
        }

        double gamepad2_right_stick = gamepad2.right_stick_x;
        if (Math.abs(gamepad2_right_stick) > 0.15) {
            robot.intake.setTurretFactor(Math.pow(gamepad2_right_stick, 3));
        }

        boolean dLB = gamepad2.left_bumper;
        boolean dRB = gamepad2.right_bumper;
        if (dLB && !pDLB) {
            schedule(new TeleopIntakeCommand(robot));
//            busyCone = false;
        } else if (dRB && !pDRB) {
            schedule(new TeleopTransferCommand(robot));
        }
        pDRB = dRB;
        pDLB = dLB;

        boolean dBA = gamepad2.a;
        boolean dBX = gamepad2.x;
        boolean dBY = gamepad2.y;
        boolean dBB = gamepad2.b;
        if (dBA && !pDBA) {
            schedule(new TeleopLiftCommand(robot, 125, LiftSubsystem.STATE.FAILED_EXTEND));
        } else if (dBX && !pDBX) {
            schedule(new TeleopLiftCommand(robot, 360, LiftSubsystem.STATE.FAILED_EXTEND));
        } else if (dBY && !pDBY) {
            schedule(new TeleopLiftCommand(robot, 610, LiftSubsystem.STATE.FAILED_EXTEND));
        } else if (dBB && !pDBB) {
            schedule(new TeleopLiftCommand(robot, 0, LiftSubsystem.STATE.FAILED_RETRACT));
        }

        robot.drivetrain.set(drive);
        robot.drivetrain.updateModules();

        boolean dRS = gamepad1.right_stick_button;
        if (dRS && !pDRS) {
            xLock = !xLock;
        }
        pDRS = dRS;

        if (xLock) {
            robot.drivetrain.leftFrontModule.setTargetRotation(PI / 4);
            robot.drivetrain.rightFrontModule.setTargetRotation(-PI / 4);
            robot.drivetrain.rightRearModule.setTargetRotation(PI / 4);
            robot.drivetrain.leftRearModule.setTargetRotation(-PI / 4);
            robot.drivetrain.updateModules();
        }

        robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();

        robot.write();

        // Telemetry
        telemetry.addData("liftPos:", robot.lift.getPos());
        telemetry.addData("liftPow:", robot.lift.power);
        telemetry.addData("liftTarget:", robot.lift.targetPosition);
        telemetry.addData("intakePos:", robot.intake.getPos());
        telemetry.addData("intakePow:", robot.intake.power);
        telemetry.addData("intakeTarget:", robot.intake.targetPosition);
        telemetry.addData("imu", FileInterface.read(FileInterface.IMU));
        telemetry.addData("intake", FileInterface.read(FileInterface.INTAKE));
        telemetry.addData("lift", FileInterface.read(FileInterface.LIFT));

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        robot.intake.extension.resetEncoder();
        robot.intake.extension.set(0);
        robot.lift.lift.resetEncoder();
        robot.lift.lift.set(0);
    }
}
