package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionLock;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
@TeleOp(name = "pos_lock")
public class PositionLockTest extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;


    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;

    Localizer localizer;


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
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.startIMUThread(this);
        }

        Pose powers = PositionLock.calculate(localizer.getPos(), new Pose(), 13);

        if (powers.x == 0 && powers.y == 0 && powers.heading == 0) {
            drivetrain.frontLeftModule.setTargetRotation(Math.PI / 8);
            drivetrain.frontRightModule.setTargetRotation(-Math.PI / 8);
            drivetrain.backRightModule.setTargetRotation(Math.PI / 8);
            drivetrain.backLeftModule.setTargetRotation(-Math.PI / 8);
        }

        drivetrain.read();
        drivetrain.set(powers);
        drivetrain.updateModules();
        drivetrain.write();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("x", localizer.getPos().x);
        telemetry.addData("y", localizer.getPos().y);
        telemetry.addData("h", localizer.getPos().heading);
        telemetry.addData("powers", powers);
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }
}
