package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.SwerveXCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.CycleCommand;
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

@Autonomous(name = "1+10 Left High")
@Config
public class Left10High extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private SleeveDetection sleeveDetection;
    private double loopTime;

    @Override
    public void runOpMode() throws InterruptedException {
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
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.PI/2), 25000, 25000, hardwareMap.voltageSensor.iterator().next().getVoltage())
//                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, 0), 5000, 5000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
//                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, 0), 5000, 5000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
//                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, 0), 5000, 5000, hardwareMap.voltageSensor.iterator().next().getVoltage())
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new PositionCommand(drivetrain, localizer, new Pose(-5, 5, 0.22899), 500, 3000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
//                                        new SwerveXCommand(drivetrain)
//                                ),
//                                new WaitCommand(0)
//
////                                new WaitCommand(2500).andThen(new SequentialCommandGroup(
////                                        new CycleCommand(lift, intake, new GrabPosition(536, 0, 0.202, 0.37, 750), LiftSubsystem.LiftState.HIGH),
////                                        new CycleCommand(lift, intake, new GrabPosition(523, 0, 0.164, 0.37, 750), LiftSubsystem.LiftState.HIGH),
////                                        new CycleCommand(lift, intake, new GrabPosition(514, 0, 0.131, 0.37, 750), LiftSubsystem.LiftState.HIGH),
////                                        new CycleCommand(lift, intake, new GrabPosition(517, 0, 0.1, 0.37, 750), LiftSubsystem.LiftState.HIGH),
////                                        new CycleCommand(lift, intake, new GrabPosition(522, 0, 0.06, 0.37, 750), LiftSubsystem.LiftState.HIGH)
//////                                        new SequentialCommandGroup(
//////                                                new LiftCommand(lift, LiftSubsystem.LiftState.HIGH)
//////                                                        .alongWith(new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED)),
//////                                                new WaitUntilCommand(lift::isWithinTolerance),
//////                                                new LatchCommand(lift, LiftSubsystem.LatchState.UNLATCHED),
//////                                                new WaitCommand(75),
//////                                                new LiftCommand(lift, LiftSubsystem.LiftState.RETRACTED)
////                                ))
//                        ),
//                        new WaitCommand(0)
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

            Pose currentPose = localizer.getPos();
            telemetry.addLine("CURRENT POSE");
            telemetry.addData("poseX", currentPose.x);
            telemetry.addData("poseY", currentPose.y);
            telemetry.addData("heading", currentPose.heading);
            telemetry.addLine("TARGET POSE");
            telemetry.addData("poseX", Globals.yummypose.x);
            telemetry.addData("poseY", Globals.yummypose.y);
            telemetry.addData("heading", Globals.yummypose.heading);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }
}
