package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Kinematics;
import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@Config
@TeleOp (name = "InverseKinematicsTest")
public class InverseKinematicsTest extends OpMode {
    private Robot robot;

    public static double x, y, l;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        double[] kinVals = Kinematics.fourbar(4, 4, robot.intake.FOURBAR_LENGTH);
        telemetry.addData("CurPos", kinVals[0]);
        telemetry.addData("CurAng", kinVals[1]);
        telemetry.update();
    }
}
