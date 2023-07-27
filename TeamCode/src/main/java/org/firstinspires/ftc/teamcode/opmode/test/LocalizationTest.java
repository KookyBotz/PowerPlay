package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//@Disabled
@TeleOp(name = "LocalizationTest")
public class LocalizationTest extends CommandOpMode {
    private ElapsedTime timer;

    private RobotHardware robot = RobotHardware.getInstance();
    private TwoWheelLocalizer localizer;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.AUTO = true;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
//        SwerveDrivetrain.imuOffset = 0;
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
//        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(this);
            robot.reset();
            localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        }

        localizer.periodic();

        Pose currentPose = localizer.getPos();
        telemetry.addData("poseX", currentPose.x);
        telemetry.addData("poseY", currentPose.y);
        telemetry.addData("heading", currentPose.heading);
        telemetry.addData("parallel", robot.parallelPod.getPosition());
        telemetry.addData("perpindicular", robot.perpindicularPod.getPosition());
        telemetry.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
}
