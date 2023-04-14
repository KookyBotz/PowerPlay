package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;

@Autonomous(name = "1+5 Left Coverage")
@Config
public class LeftCoverageAuto extends LinearOpMode {

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

            telemetry.addLine("Left Coverage Auto");
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

//        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // B3
                        new PositionCommand(drivetrain, localizer, new Pose(0, 59.35, 0.235), 250, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),
                        // B2 pickup
                        new PositionCommand(drivetrain, localizer, new Pose(0, 43.5, -0.235), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),
                        // B2
                        new PositionCommand(drivetrain, localizer, new Pose(-4, 42.5, -0.235), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),
                        //B1 pickup
                        new PositionCommand(drivetrain, localizer, new Pose(1, 43.5, -0.235), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),
                        //B1
                        new PositionCommand(drivetrain, localizer, new Pose(0, 43.5, 0.235), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),
                        //A2 pickup
                        new PositionCommand(drivetrain, localizer, new Pose(0, 43.5, -0.235), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),

                        //A2
                        new PositionCommand(drivetrain, localizer, new Pose(0, 43.5, 2), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),

                        //C2 pickup
                        new PositionCommand(drivetrain, localizer, new Pose(0, 54.5, 0), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),

                        //C2
                        new PositionCommand(drivetrain, localizer, new Pose(-28, 52, -Math.PI / 4), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),

                        //C2 pickup
                        new PositionCommand(drivetrain, localizer, new Pose(0, 54.5, 0), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500),

                        //C2
                        new PositionCommand(drivetrain, localizer, new Pose(-28, 52, -Math.PI / 4), 250, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new WaitCommand(500)
                )
        );

        robot.stopCameraStream();
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
