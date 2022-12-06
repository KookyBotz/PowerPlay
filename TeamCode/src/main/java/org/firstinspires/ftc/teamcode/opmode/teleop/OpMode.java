package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

@Config
@TeleOp(name = "OpMode👌👌😍🎶🎶😎")
public class OpMode extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;

    boolean xLock = false;
    boolean pDRS = false;

    boolean pDLB = false;
    boolean pDRB = false;
    boolean pDDL = false;
    boolean pDRT = false;
    boolean pDLT = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.intake.extension.set(-0.4);
        robot.lift.lift.set(-0.3);
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

        boolean dDL = gamepad2.dpad_left;
        if (dDL && !pDDL) schedule(); // TODO: Add back teleop cycle command
        pDDL = dDL;

        boolean dLT = (gamepad2.left_trigger > 0.3);
        boolean dRT = (gamepad2.right_trigger > 0.3);
        if (dLT && !pDLT) {
            schedule(); // TODO: Open Claw
        } else if (dRT && !pDRT) {
            schedule(); // TODO: Close Claw
        }

        double gamepad2_left_stick_y = gamepad2.left_stick_y;
        if (Math.abs(gamepad2_left_stick_y) > 0.15) {
            robot.intake.setFourbarFactor(Math.pow(gamepad2_left_stick_y, 3));
        }

        double gamepad2_left_stick_x = gamepad2.left_stick_x;
        if (Math.abs(gamepad2_left_stick_x) > 0.01) {
            robot.intake.setSlideFactor(Math.pow(gamepad2_left_stick_x, 3));
        }

        double gamepad2_right_stick = gamepad2.right_stick_x;
        if (Math.abs(gamepad2_right_stick) > 0.15) {
            robot.intake.setTurretFactor(Math.pow(gamepad2_right_stick, 3));
        }

        boolean dLB = gamepad2.left_bumper;
        boolean dRB = gamepad2.right_bumper;
        if (dLB && !pDLB) {
            schedule(); // TODO: Go to grab position
        } else if (dRB && !pDRB) {
            schedule(); // TODO: Go to transfer position
        }
        pDRB = dRB;
        pDLB = dLB;

        //TODO: these need to all be ra detectors lmfao
        if (gamepad2.a) {
            schedule(); // TODO: LOW
        } else if (gamepad2.x) {
            schedule(); // TODO: Medium
        } else if (gamepad2.y) {
            schedule(); // TODO: High
        } else if (gamepad2.b) {
            schedule(); // TODO: Retract
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
