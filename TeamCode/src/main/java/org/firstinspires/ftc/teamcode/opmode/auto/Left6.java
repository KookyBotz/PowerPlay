package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.HighPoleAutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionLockCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Autonomous(name = "Auto Left")
@Config
public class Left6 extends LinearOpMode {

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
        CommandScheduler.getInstance().reset();
        Globals.AUTO = false;
        Globals.USING_IMU = true;
        Globals.USE_WHEEL_FEEDFORWARD = true;

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

            telemetry.addLine("1+5 LEFT SIDE HIGH");
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }


        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        GrabPosition[] grabPositions = new GrabPosition[]{
                new GrabPosition(570, 0, 0.173, 0.37, 0),
                new GrabPosition(555, 0, 0.14, 0.37, 0),
                new GrabPosition(550, 0, 0.105, 0.37, 0),
                new GrabPosition(550, 0, 0.075, 0.37, 0),
                new GrabPosition(560, 0, 0.04, 0.37, 0)
        };

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(3.5, 64, 0), 500, robot.getVoltage()),
                        new InstantCommand(()->Globals.USE_WHEEL_FEEDFORWARD = false),
                        new PositionLockCommand(drivetrain, localizer, new Pose(3.5, 60.8, 0.24), () -> false, robot.getVoltage()),
//                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, grabPositions[0], LiftSubsystem.LiftState.HIGH),
//                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, grabPositions[1], LiftSubsystem.LiftState.HIGH),
//                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, grabPositions[2], LiftSubsystem.LiftState.HIGH),
//                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, grabPositions[3], LiftSubsystem.LiftState.HIGH),
//                        new HighPoleAutoCycleCommand(drivetrain, lift, intake, grabPositions[4], LiftSubsystem.LiftState.HIGH),
                        new InstantCommand(() -> endtime = timer.milliseconds())
                )
        );

        robot.reset();
        timer = new ElapsedTime();

        while (opModeIsActive()) {
            robot.read(drivetrain, intake, lift);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain, intake, lift);
            localizer.periodic();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("end", endtime);
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }
}