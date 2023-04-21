package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.HighPoleAutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.SwerveXCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;

@Autonomous(name = "Time to cook")
@Config
public class Left10HighOptimizedPath extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private SleeveDetection sleeveDetection;
    private double loopTime;
    private double endtime = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        PositionCommand.xController.setPIDF(0.02, 0, 0.03, 0);
        PositionCommand.yController.setPIDF(0.02, 0, 0.03, 0);
        PositionCommand.hController.setPIDF(0.3, 0, 0.05, 0);


        CommandScheduler.getInstance().reset();
        Globals.SIDE = Globals.Side.LEFT;
        Globals.AUTO = true;
        Globals.USING_IMU = true;

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
            for (SwerveModule module : drivetrain.modules) {
                module.setTargetRotation(Math.PI / 2);
            }
            drivetrain.updateModules();

            telemetry.addLine("1+10 LEFT SIDE HIGH");
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

//        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(0, 59.35, 0.235), 250, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(540, 0, 0.163, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(522, 0, 0.135, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(513, 0, 0.1, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(512, 0, 0.07, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(515, 0, 0.035, 0.37, 20), LiftSubsystem.LiftState.HIGH)
                                ),
                                new SwerveXCommand(drivetrain)
                        ),
                        new PositionCommand(drivetrain, localizer, new Pose(-17, 49.5, Math.PI / 2), 0, 750, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        // we just need to get slightly close before we can run the next path, dont actually care where
                        new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 7),
                        new PositionCommand(drivetrain, localizer, new Pose(-69, 52, Math.PI / 2), 0, 3000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 0.25),
                        new PositionCommand(drivetrain, localizer, new Pose(-69, 62.7, Math.PI - 0.23), 250, 800, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(540, 0, 0.172, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(522, 0, 0.139, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(513, 0, 0.106, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(512, 0, 0.075, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(515, 0, 0.035, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                        new LiftCommand(lift, LiftSubsystem.LiftState.HIGH),
                                        new WaitCommand(150),
                                        new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                                        new WaitUntilCommand(lift::isWithinTolerance),
                                        new WaitCommand(25),
                                        new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                        new WaitCommand(75),
                                        new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
                                ),
                                new SwerveXCommand(drivetrain)
                        ),
                        new PositionCommand(drivetrain, localizer, new Pose(-69, 28, Math.PI / 2), 0, 1000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(-45, 28, Math.PI / 2), 0, 1000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new InstantCommand(() -> endtime = timer.milliseconds())
                )
        );

        robot.reset();

        while (opModeIsActive()) {
            if (timer == null) {
                timer = new ElapsedTime();
            }
            robot.read(drivetrain, intake, lift);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain, intake, lift);
            localizer.periodic();

            telemetry.addData("encoder pod", intake.getPos());
            telemetry.addData("time", endtime);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }
}