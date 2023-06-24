package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_MANUAL_FACTOR;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.INTAKE_MAX;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands.TeleOpAutoDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands.TeleOpAutoGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands.TeleOpExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands.TeleOpStackCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.powerplay.Junction;

import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    private final PIDFController hController = new PIDFController(0.5, 0, 0.1, 0);

    public static double fw_r = 4;
    public static double str_r = 4;
    private boolean lock_robot_heading = false;

    GamepadEx gamepadEx, gamepadEx2;
    Localizer localizer;

    public static boolean autoGrabActive = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USING_IMU = true;
        Globals.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        intake = new IntakeSubsystem(robot);
        lift = new LiftSubsystem(robot);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        BooleanSupplier depositSupplier = () -> gamepad1.right_bumper;

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new TeleOpAutoGrabCommand(intake, () -> gamepad2.right_trigger > 0.5)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new TeleOpAutoDepositCommand(lift, intake, Junction.HIGH, depositSupplier)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new TeleOpAutoDepositCommand(lift, intake, Junction.MEDIUM, depositSupplier)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new TeleOpAutoDepositCommand(lift, intake, Junction.LOW, depositSupplier)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> schedule(new TeleOpAutoDepositCommand(lift, intake, Junction.GROUND, depositSupplier)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> schedule(new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.LATCHED))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> schedule(new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new InstantCommand(() -> lift.setReady(true))));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new InstantCommand(() -> intake.changeStackHeight(1))));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new InstantCommand(() -> intake.changeStackHeight(-1))));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .toggleWhenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN))),
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.FALLEN)),
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN)))));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .toggleWhenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN))),
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.FALLEN)),
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INWARDS)),
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                        new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN)))));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new TeleOpStackCommand(intake)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new TeleOpExtendCommand(lift, intake)));
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(this);
            fw = new SlewRateLimiter(fw_r);
            str = new SlewRateLimiter(str_r);
        }

        robot.read(drivetrain, intake, lift);

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle() + Math.PI;


        if (gamepad1.right_stick_y > 0.25) {
            lock_robot_heading = true;
            targetHeading = Math.PI + SwerveDrivetrain.imuOffset;
        }
        if (gamepad1.right_stick_y < -0.25) {
            lock_robot_heading = true;
            targetHeading = 0 + SwerveDrivetrain.imuOffset;
        }

        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(turn) > 0.002) {
            lock_robot_heading = false;
        }

        double error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -hController.calculate(0, error) * 12.4 / robot.getVoltage();

        if (Math.abs(headingCorrection) < 0.01) {
            headingCorrection = 0;
        }

        SwerveDrivetrain.maintainHeading =
                (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                        Math.abs(gamepad1.left_stick_y) < 0.002 &&
                        Math.abs(turn) < 0.002) &&
                        Math.abs(headingCorrection) < 0.02;


        double rotationAmount = (Globals.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose drive = new Pose(
                new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                lock_robot_heading ? headingCorrection :
                        joystickScalar(turn, 0.01)
        );

        drive = new Pose(
                fw.calculate(drive.x),
                str.calculate(drive.y),
                drive.heading
        );

        double leftY = gamepadEx2.getRightY();
        if (Math.abs(leftY) > 0.1) {
            double slideAddition = INTAKE_MANUAL_FACTOR * leftY;
            double newPosition = intake.getPos() + slideAddition;
            if (newPosition >= 0) {
                intake.setSlideFactor(joystickScalar(leftY, 0.1));
            }
        }

        if (gamepad1.dpad_up) {
            schedule(new InstantCommand(() -> intake.retractReset()));
        }

        if (gamepad2.y) {
            schedule(new InstantCommand(() -> lift.retractReset()));
        }

        robot.loop(drive, drivetrain, intake, lift);
        robot.write(drivetrain, intake, lift);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("height", intake.stackHeight);
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}
