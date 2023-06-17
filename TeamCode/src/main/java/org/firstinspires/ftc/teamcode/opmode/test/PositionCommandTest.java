package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
@TeleOp(name = "pos command")
public class PositionCommandTest extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;


    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;

    TwoWheelLocalizer localizer;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USE_WHEEL_FEEDFORWARD = true;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        drivetrain.frontLeftModule.setTargetRotation(Math.PI / 2);
        drivetrain.frontRightModule.setTargetRotation(Math.PI / 2);
        drivetrain.backRightModule.setTargetRotation(Math.PI / 2);
        drivetrain.backLeftModule.setTargetRotation(Math.PI / 2);

        while (!this.isStarted()) {
            drivetrain.read();
            drivetrain.updateModules();
            robot.clearBulkCache();

            telemetry.addData("t", drivetrain.frontLeftModule.getTargetRotation());
            telemetry.addData("c", drivetrain.frontLeftModule.getModuleRotation());
            telemetry.update();
        }
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.startIMUThread(this);
            localizer.setPoseEstimate(new Pose2d(0, 0, 0));
            schedule(new PositionCommand(drivetrain, localizer, new Pose(0, 60, 0), 500, 12.5));
        }


        drivetrain.read();
        drivetrain.updateModules();
        drivetrain.write();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("x", localizer.getPos().x);
        telemetry.addData("y", localizer.getPos().y);
        telemetry.addData("h", localizer.getPos().heading);
        telemetry.addData("t", drivetrain.frontLeftModule.getTargetRotation());
        telemetry.addData("c", drivetrain.frontLeftModule.getModuleRotation());
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }
}
