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
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PrecisePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
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

@Autonomous(name = "Wartime Auto Right")
@Config
public class Left5HighRight extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private double loopTime;
    private double endtime = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        PositionCommand.xController.setPIDF(0.02, 0, 0.03, 0);
        PositionCommand.yController.setPIDF(0.02, 0, 0.03, 0);
        PositionCommand.hController.setPIDF(0.3, 0, 0.05, 0);


        CommandScheduler.getInstance().reset();
        Globals.SIDE = Globals.Side.RIGHT;
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
                module.setTargetRotation(Math.PI / 2.25);
            }
            drivetrain.updateModules();

            telemetry.addLine("1+5 RIGHT SIDE HIGH");
            telemetry.addData("position", robot.sleeveDetection.getPosition());
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

        SleeveDetection.ParkingPosition position = robot.sleeveDetection.getPosition();

        final Pose[] cycleTarget = new Pose[]{new Pose()};

        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(2.25, -59.2, -0.235), 1000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new InstantCommand(() -> cycleTarget[0] = localizer.getPos()),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PrecisePositionCommand(drivetrain, localizer, cycleTarget, 0, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(540, 0, 0.163, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new PrecisePositionCommand(drivetrain, localizer, cycleTarget, 0, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(522, 0, 0.135, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new PrecisePositionCommand(drivetrain, localizer, cycleTarget, 0, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(513, 0, 0.1, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                        new PrecisePositionCommand(drivetrain, localizer, cycleTarget, 0, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(512, 0, 0.07, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                        new PrecisePositionCommand(drivetrain, localizer, cycleTarget, 0, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(515, 0, 0.035, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                        new PrecisePositionCommand(drivetrain, localizer, cycleTarget, 0, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                                        new LiftCommand(lift, LiftSubsystem.LiftState.HIGH),
                                        new WaitCommand(150),
                                        new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                                        new WaitUntilCommand(lift::isWithinTolerance),
                                        new WaitCommand(25),
                                        new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                        new WaitCommand(75),
                                        new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
                                )
                        ),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -29.35, -Math.PI / 2), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new PositionCommand(drivetrain, localizer,
                                (position.equals(SleeveDetection.ParkingPosition.LEFT) ? new Pose(-22, -29.35, -Math.PI / 2) :
                                        (position.equals(SleeveDetection.ParkingPosition.CENTER) ? new Pose(0, -29.35, -Math.PI / 2) :
                                                new Pose(25, -29.35, -Math.PI / 2))), 2000, 2000,
                                hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new InstantCommand(() -> endtime = timer.milliseconds()),
                        new InstantCommand(this::requestOpModeStop)
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

//            telemetry.addData("encoder pod", intake.getPos());
//            telemetry.addData("time", endtime);
            telemetry.addLine(cycleTarget[0].toString());
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }
}