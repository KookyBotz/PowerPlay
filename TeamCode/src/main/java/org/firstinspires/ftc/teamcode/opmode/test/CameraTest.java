package org.firstinspires.ftc.teamcode.opmode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
@Autonomous(name = "CameraTest") @Config
public class CameraTest extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    @Override
    public void runOpMode() {
        Globals.AUTO = true;
        Globals.SIDE = Globals.Side.RIGHT;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        FtcDashboard.getInstance().startCameraStream(robot.backCamera, 30);
        waitForStart();
        while (opModeIsActive());
        robot.stopCameraStream();}}
