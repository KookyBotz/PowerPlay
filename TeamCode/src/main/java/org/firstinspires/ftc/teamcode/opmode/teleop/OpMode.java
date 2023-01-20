package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.ConeVomitHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.ConeVomitMidCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

@Config
@TeleOp(name = "OpMode👌👌😍🎶🎶😎😜😭🥰😈👺👺🤣🤣😕😜😭🥰🥰😘")
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

    private boolean busy = false;

    private boolean pD1A = false;
    private boolean pD1Y = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.intake.setFourbar(IntakeSubsystem.fourbar_transition);
        robot.intake.offset2 = 0;
        robot.intake.update(IntakeSubsystem.PivotState.FLAT);
        robot.intake.update(IntakeSubsystem.TurretState.INTAKE);
        robot.intake.update(IntakeSubsystem.ClawState.OPEN);

        robot.intake.extension.set(0.4);
        robot.lift.lift.set(0.4);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.startIMUThread(this);
        }

        robot.read();
        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        // Drivetrain
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3)),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3))).rotate(robot.getAngle() - SwerveDrivetrain.imuOff),
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier
        );

        if (gamepad1.left_bumper) {
            SwerveDrivetrain.imuOff = robot.getAngle();
        }

        boolean d1A = gamepad1.a;
        boolean d1Y = gamepad1.y;
        if (d1A && !pD1A) {
            // decrease
            robot.intake.adjustPivotOffset(-0.03);
        } else if (d1Y && !pD1Y) {
            // increase
            robot.intake.adjustPivotOffset(0.03);
        }
        pD1A = d1A;
        pD1Y = d1Y;

        // Gamepad2
        if (gamepad2.dpad_up) {
            robot.lift.setSlideFactor(1);
        } else if (gamepad2.dpad_down) {
            robot.lift.setSlideFactor(-1);
        }

        if (gamepad2.dpad_left && !busy) {
            busy = true;
            CommandScheduler.getInstance().schedule(new ConeVomitHighCommand(robot, (b) -> busy = b));
        }
        if (gamepad2.dpad_right && !busy) {
            busy = true;
            CommandScheduler.getInstance().schedule(new ConeVomitMidCommand(robot, (b) -> busy = b));
        }

        boolean dLT = (gamepad2.left_trigger > 0.3);
        boolean dRT = (gamepad2.right_trigger > 0.3);
        if (dLT && !pDLT) {
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
        } else if (dRT && !pDRT) {
            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)));
        }
        pDLT = dLT;
        pDRT = dRT;

        double gamepad2_left_stick_y = gamepad2.left_stick_y;
        if (Math.abs(gamepad2_left_stick_y) > 0.2) {
            robot.intake.setFourbarFactor(joystickScalar(gamepad2_left_stick_y, 0.2));
        }

        double gamepad2_left_stick_x = gamepad2.left_stick_x;
        if (Math.abs(gamepad2_left_stick_x) > 0.1) {
            robot.intake.setSlideFactor(joystickScalar(-gamepad2_left_stick_x, 0.1));
        }

        double gamepad2_right_stick = gamepad2.right_stick_x;
        if (Math.abs(gamepad2_right_stick) > 0.15) {
            robot.intake.setTurretFactor(joystickScalar(gamepad2_right_stick, 0.15));
        }

        boolean dLB = gamepad2.left_bumper;
        boolean dRB = gamepad2.right_bumper;
        if (dLB && !pDLB) {
            schedule(new TeleopIntakeCommand(robot));
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
            schedule(
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                    new IntakePositionCommand(robot.intake, -5, 6000, 2500, 20, 3000, IntakeSubsystem.STATE.FAILED_RETRACT),
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.SCORE)),
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.SCORE)),
                    new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE))
            );
        } else if (dBX && !pDBX) {
            schedule(new TeleopLiftCommand(robot, 340, LiftSubsystem.STATE.FAILED_EXTEND));
        } else if (dBY && !pDBY) {
            schedule(new TeleopLiftCommand(robot, 580, LiftSubsystem.STATE.FAILED_EXTEND));
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
        telemetry.addData("imu", SwerveDrivetrain.imuOff);

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

    private double joystickScalar(double num, double min) {
        return Math.signum(num) * min + (1 - min) * Math.pow(Math.abs(num), 3) * Math.signum(num);
    }
}
