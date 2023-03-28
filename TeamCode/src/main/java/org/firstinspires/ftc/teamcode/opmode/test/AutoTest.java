package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

@Config
@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private TwoWheelLocalizer localizer;
    private SwerveDrivetrain drivetrain;

    public static double x = 10;
    public static double y = 0;
    public static double heading = 0;

    private ElapsedTime timer;
    private Pose currentPose = new Pose();
    private Pose targetPose = new Pose(10, 10, Math.PI / 2);

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.AUTO = true;
        Globals.USING_IMU = true;
        Globals.SIDE = Globals.Side.LEFT;

        robot.init(hardwareMap, telemetry);
        localizer = new TwoWheelLocalizer(robot);
        drivetrain = new SwerveDrivetrain(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        while (!isStarted()) {
            robot.read(drivetrain, null, null);
            for (SwerveModule module : drivetrain.modules) {
                module.setTargetRotation(Math.PI / 2);
            }
            drivetrain.updateModules();

            telemetry.addLine("test auto here we go");
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            robot.write(drivetrain, null, null);
        }

        waitForStart();
        robot.startIMUThread(this);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
//                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.PI), 3000, 3000, robot.voltageSensor.getVoltage())
                        new PositionCommand(drivetrain, localizer, new Pose(0, -20, heading), 3000, 3000, robot.voltageSensor.getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(20, -20, heading), 3000, 3000, robot.voltageSensor.getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(20, 0, heading), 3000, 3000, robot.voltageSensor.getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, heading), 3000, 3000, robot.voltageSensor.getVoltage())
                )
        );

        robot.reset();

        while (opModeIsActive()) {
            robot.read(drivetrain, null, null);

            CommandScheduler.getInstance().run();
            drivetrain.updateModules();
            localizer.periodic();

            currentPose = localizer.getPos();
            telemetry.addData("poseX", currentPose.x);
            telemetry.addData("poseY", currentPose.y);
            telemetry.addData("heading", currentPose.heading);
            telemetry.update();

            robot.write(drivetrain, null, null);
            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
