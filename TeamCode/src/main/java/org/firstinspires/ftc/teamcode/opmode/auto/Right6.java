package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionLockCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.SixConeAutoCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;

import java.util.function.DoubleSupplier;

@Autonomous(name = "6 Right ➡")
@Config
public class Right6 extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private double loopTime;
    private double endtime;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();
        Globals.AUTO = true;
        Globals.USING_IMU = true;
        Globals.SIDE = Globals.Side.LEFT;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        intake = new IntakeSubsystem(robot);
        lift = new LiftSubsystem(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        while (!isStarted()) {
            robot.read(drivetrain, intake, lift);
            drivetrain.frontLeftModule.setTargetRotation(0);
            drivetrain.frontRightModule.setTargetRotation(0);
            drivetrain.backLeftModule.setTargetRotation(0);
            drivetrain.backRightModule.setTargetRotation(0);
            drivetrain.updateModules();

            telemetry.addLine("1+5 RIGHT SIDE HIGH");
            telemetry.addData("x", localizer.getPos().x);
            telemetry.addData("y", localizer.getPos().y);
            telemetry.addData("position", robot.sleeveDetection.getPosition());
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

        SleeveDetection.ParkingPosition parkingPosition = robot.sleeveDetection.getPosition();

        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        robot.reset();
        timer = new ElapsedTime();

        DoubleSupplier time_left = () -> 30 - timer.seconds();
        SixConeAutoCommand sixConeAutoCommand = new SixConeAutoCommand(robot, localizer, drivetrain, intake, lift, time_left, parkingPosition, true,true);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(-66.5, 5.5, 0), 500, 2500, robot.getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(-60.8, 5.5, -(0.24 - Math.PI / 2)), 0, 2000, robot.getVoltage()),
                        new InstantCommand(() -> PositionLockCommand.setTargetPose(new Pose(-60.8, 5.5, -(0.26 - Math.PI / 2)))),
                        new PositionLockCommand(drivetrain, localizer, sixConeAutoCommand::isFinished, robot.getVoltage())
                                .alongWith(new WaitCommand(1000).andThen(sixConeAutoCommand)),
                        new InstantCommand(() -> endtime = timer.seconds()))
        );


        while (opModeIsActive()) {
            robot.read(drivetrain, intake, lift);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain, intake, lift);
            localizer.periodic();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addLine(sixConeAutoCommand.getTelemetry());
            telemetry.addData("endtime", endtime);
            telemetry.addData("x", localizer.getPos().x);
            telemetry.addData("y", localizer.getPos().y);
            telemetry.addData("h", localizer.getPos().heading);
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }

    public static int getModifier() {
        return Globals.SIDE == Globals.Side.LEFT ? 1 : -1;
    }
}