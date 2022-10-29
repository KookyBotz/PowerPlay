package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionState;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.AsymetricMotionProfile;

@Config
@TeleOp(name = "ProfilingTest2")
public class ProfilingTest2 extends LinearOpMode {
    MotorEx m;
    AsymetricMotionProfile profile;
    PIDController controller;
    ElapsedTime timer;

    public static double P = 0.0, I = 0.0, D = 0.0;
    public static MotionConstraints constraints;
    public static MotionState state;

    public static double currentPos = 0.0;
    public static double startPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        m = new MotorEx(hardwareMap, "motor1");
        m.resetEncoder();
        currentPos = m.encoder.getPosition();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            // sus
        }
    }
}
