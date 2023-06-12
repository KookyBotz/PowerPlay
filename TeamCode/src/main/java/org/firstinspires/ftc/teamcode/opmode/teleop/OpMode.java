package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand.hController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.TeleOpAutoGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

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

    public static double fw_r = 4;
    public static double str_r = 4;

    GamepadEx gamepadEx;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        intake = new IntakeSubsystem(robot);
        lift = new LiftSubsystem(robot);
        gamepadEx = new GamepadEx(gamepad1);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()->schedule(new TeleOpAutoGrabCommand(intake)));
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            try {
                robot.reset();
                robot.startIMUThread(this);
            } catch (Exception ignored) {
            }

            fw = new SlewRateLimiter(fw_r);
            str = new SlewRateLimiter(str_r);
        }

        robot.read(drivetrain, intake, lift);

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle();

        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        boolean lock_robot_heading = Math.abs(turn) < 0.002;

        if (lock_robot_heading && !pHeadingLock) targetHeading = robot.getAngle();

        double error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -hController.calculate(0, error);

        if (Math.abs(headingCorrection) < 0.01) {
            headingCorrection = 0;
        }

        pHeadingLock = lock_robot_heading;

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


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        robot.loop(drive, drivetrain, intake, lift);
        robot.write(drivetrain, intake, lift);

        loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("x", intake.fourbarMotionState.x);
        telemetry.addData("v", intake.fourbarMotionState.v);
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 3);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}
