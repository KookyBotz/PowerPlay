package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand.hController;
import static org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand.relDistanceToTarget;
import static org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand.xController;
import static org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand.yController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.SwerveXCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.HighPoleAutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.GroundScoreCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.IntermediateStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.LowScoreCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem.ClawState;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private ElapsedTime timer;
    private ElapsedTime timer2;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private TwoWheelLocalizer localizer;

    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    private SlewRateLimiter t;

    public static double fw_r = 4;
    public static double str_r = 4;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


        Globals.AUTO = false;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            try {
                robot.reset();
                robot.startIMUThread(this);
                localizer.setPoseEstimate(new Pose2d(0, 0, 0));
            } catch (Exception ignored) {
            }

            fw = new SlewRateLimiter(fw_r);
            str = new SlewRateLimiter(str_r);
        }

        drivetrain.read();

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle();

        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        boolean lock_robot_heading = Math.abs(turn) < 0.002;

        if (lock_robot_heading && !pHeadingLock) targetHeading = robot.getAngle();

        double error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -hController.calculate(0, error);
//        headingCorrection += 0.02 * Math.signum(headingCorrection);
        if(Math.abs(headingCorrection) < 0.01){
            headingCorrection = 0;
        }

        pHeadingLock = lock_robot_heading;

        SwerveDrivetrain.maintainHeading =
                (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                        Math.abs(gamepad1.left_stick_y) < 0.002 &&
                        Math.abs(turn) < 0.002 &&
                        Math.abs(headingCorrection) < 0.02);


        double rotationAmount = (Globals.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose drive = new Pose(
                new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                lock_robot_heading ? headingCorrection : joystickScalar(turn, 0.01)
        );


        CommandScheduler.getInstance().run();

        if (gamepad1.a) {
            Pose robotPose = localizer.getPos();
            Pose deltaPose = relDistanceToTarget(robotPose, new Pose());
            Pose powers = new Pose(
                    xController.calculate(0, deltaPose.x),
                    yController.calculate(0, deltaPose.y),
                    hController.calculate(0, deltaPose.heading)
            );

            double max_power = 0.5;
            double max_heading = 0.5;

            double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
            double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
            double x_power = -x_rotated < -max_power ? -max_power :
                    Math.min(-x_rotated, max_power);
            double y_power = -y_rotated < -max_power ? -max_power :
                    Math.min(-y_rotated, max_power);
            double heading_power = powers.heading;

            heading_power = Math.max(Math.min(max_heading, heading_power), -max_heading);

            drive = new Pose(-y_power + 0.04 * Math.signum(-y_power), x_power + 0.04 * Math.signum(x_power), -heading_power);

            targetHeading = 0;
        }

        if (gamepad1.b) {
            localizer.setPoseEstimate(new Pose2d(0, 0, robot.getAngle()));
        }

        drive = new Pose(
                fw.calculate(drive.x),
                str.calculate(drive.y),
                drive.heading
        );


        drivetrain.set(drive);
        drivetrain.updateModules();
        drivetrain.write();

        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("correction", headingCorrection);
        telemetry.addData("pos", localizer.getPos().toString());
        telemetry.addData("perp", robot.perpindicularPod.getPosition());
        telemetry.addData("par", robot.parallelPod.getPosition());
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
