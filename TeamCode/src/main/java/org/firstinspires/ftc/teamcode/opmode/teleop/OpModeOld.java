package org.firstinspires.ftc.teamcode.opmode.teleop;

import static java.lang.Math.PI;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

@Config
@TeleOp(name = "OpMode👌👌😍🎶🎶😎😜😭🥰😈👺👺🤣🤣😕😜😭🥰🥰😘")
public class OpModeOld extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;

    private boolean xLock = false;

    public static double decelConstraint = 2000;
    public static double accelConstraint = 7500;
    public static double velocity = 6500;

    public static boolean usingIMU = false;
    private boolean pHasCone = false;

    private final GamepadEx gamepadEx2 = new GamepadEx(gamepad2),
                            gamepadEx1 = new GamepadEx(gamepad1);

    private boolean rumble = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot.intake.setFourbar(IntakeSubsystem.fourbar_transition);
        robot.intake.pivotOffset = 0;
//        robot.intake.update(IntakeSubsystem.PivotState.FLAT);
//        robot.intake.update(IntakeSubsystem.TurretState.INTAKE);
        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.lift.update(LiftSubsystem.LatchState.UNLATCHED);

//        robot.extension.set(0.4);
//        robot.lift.lift1.set(0.2);
//        robot.lift.lift2.set(0.2);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            if (usingIMU) {
                robot.startIMUThread(this);
            }
            SwerveDrivetrain.imuOffset = -Math.PI / 2;
        }

        if (timer.seconds() < 45 && timer.seconds() > 30 && rumble) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            rumble = false;
        }

        if (timer.seconds() < 30 && !rumble) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            rumble = true;
        }

        robot.read();

        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        // Drivetrain
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3)),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3))).rotate(robot.getAngle() - SwerveDrivetrain.imuOffset),
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier * ((robot.intake.isExtended) ? 0.5 : 1)
        );

        if (gamepadEx1.wasJustPressed(Button.LEFT_BUMPER)) {
            SwerveDrivetrain.imuOffset = robot.getAngle() + Math.PI;
        }
//
//        if (gamepadEx1.wasJustPressed(Button.A)) {
//            robot.intake.adjustPivotOffset(-0.03);
//            robot.intake.update(robot.intake.pivotState);
//        } else if (gamepadEx1.wasJustPressed(Button.Y)) {
//            robot.intake.adjustPivotOffset(0.03);
//            robot.intake.update(robot.intake.pivotState);
//        }
//
//        if (gamepadEx1.wasJustPressed(Button.RIGHT_BUMPER)) {
//            if (robot.intake.pivotState.equals(IntakeSubsystem.PivotState.SCORE) && robot.intake.fourbarState == (IntakeSubsystem.FourbarState.SCORE)) {
//                schedule(new LowScoreCommand(robot));
//            } else {
//                schedule(new TeleopLiftCommand(robot, 0, new Constraints(6500, 7500, decelConstraint), LiftSubsystem.STATE.FAILED_RETRACT));
//            }
//        }
//
//        if (gamepadEx2.wasJustPressed(Button.DPAD_LEFT)) {
//            robot.lift.update(LiftSubsystem.LatchState.UNLATCHED);
//        } else if (gamepadEx2.wasJustPressed(Button.DPAD_RIGHT)) {
//            robot.lift.update(LiftSubsystem.LatchState.LATCHED);
//        }
//
//        double slideFactor = (gamepadEx2.getButton(Button.DPAD_UP) ? 0.03 : 0) + (gamepadEx2.getButton(Button.DPAD_DOWN) ? -0.03 : 0);
//        if (slideFactor != 0) {
//            robot.lift.setSlideFactor(slideFactor);
//        }
//
//        if (robot.intake.hasCone() && !pHasCone) {
//            schedule(new TeleopTransferCommand(robot));
//        }
//
//        if (gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3) {
//            if (robot.intake.pivotState.equals(IntakeSubsystem.PivotState.SCORE) && robot.intake.fourbarState == (IntakeSubsystem.FourbarState.SCORE)) {
//                schedule(new LowScoreCommand(robot));
//            } else {
//                schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)));
//            }
//        } else if (gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3) {
//            schedule(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)));
//        }
//
//        double leftY = gamepadEx2.getLeftY();
//        if (Math.abs(leftY) > 0.1) {
//            robot.intake.setSlideFactor(joystickScalar(-leftY, 0.1));
//        }
//
//        double leftX = gamepadEx2.getLeftX();
//        if (Math.abs(leftX) > 0.4) {
//            robot.intake.setFourbarFactor(joystickScalar(leftX, 0.4));
//        }
//
//        double rightX = gamepadEx2.getRightX();
//        if (Math.abs(rightX) > 0.15) {
//            robot.intake.setTurretFactor(joystickScalar(rightX, 0.15));
//        }
//
//        if (gamepadEx2.wasJustPressed(Button.LEFT_BUMPER)) {
//            schedule(new TeleopIntakeCommand(robot));
//        } else if (gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER)) {
//            schedule(new TeleopIntakeCommand(robot));
//        }
//
//        if (gamepadEx2.wasJustPressed(Button.A)) {
//            schedule(new LowPresetCommand(robot));
//        } else if (gamepadEx2.wasJustPressed(Button.X)) {
//            schedule(new TeleopLiftCommand(robot, robot.lift.getHeightMid(), new Constraints(6500, 7500, decelConstraint), LiftSubsystem.STATE.FAILED_EXTEND));
//        } else if (gamepadEx2.wasJustPressed(Button.Y)) {
//            schedule(new TeleopLiftCommand(robot, robot.lift.getHeightHigh(), new Constraints(6500, 7500, decelConstraint), LiftSubsystem.STATE.FAILED_EXTEND));
//        } else if (gamepadEx2.wasJustPressed(Button.B)) {
//            schedule(new TeleopLiftCommand(robot, 0, new Constraints(6500, 7500, decelConstraint), LiftSubsystem.STATE.FAILED_RETRACT));
//        }
//
//        if (gamepadEx1.wasJustPressed(Button.X)) {
//            robot.intake.update(IntakeSubsystem.PivotState.DOWN);
//            robot.intake.update(IntakeSubsystem.FourbarState.DOWN);
//        }
//
//        if (gamepadEx1.wasJustPressed(Button.RIGHT_STICK_BUTTON)) {
//            xLock = !xLock;
//        }

        if (xLock) {
            robot.drivetrain.frontLeftModule.setTargetRotation(PI / 4);
            robot.drivetrain.frontRightModule.setTargetRotation(-PI / 4);
            robot.drivetrain.backRightModule.setTargetRotation(PI / 4);
            robot.drivetrain.backLeftModule.setTargetRotation(-PI / 4);
            robot.drivetrain.updateModules();
        }

        robot.drivetrain.set(drive);
        robot.drivetrain.updateModules();

        robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();

        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
//        robot.intake.extension.resetEncoder();
//        robot.intake.extension.set(0);
//        robot.lift.liftEncoder.resetEncoder();
//        robot.lift.lift1.set(0);
//        robot.lift.lift2.set(0);
    }

    private double joystickScalar(double num, double min) {
        return Math.signum(num) * min + (1 - min) * Math.pow(Math.abs(num), 3) * Math.signum(num);
    }
}
