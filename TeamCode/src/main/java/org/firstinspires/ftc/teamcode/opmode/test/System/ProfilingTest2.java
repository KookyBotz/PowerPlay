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
    public static MotionConstraints constraints = new MotionConstraints(0, 0, 0);

    public static double startPos = 0.0;
    public static double currentPos = 0.0;
    public static double finalPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        m = new MotorEx(hardwareMap, "motor1");
        m.resetEncoder();
        currentPos = m.encoder.getPosition();
        constraints = new MotionConstraints(10, 1, 0.5);
        timer = new ElapsedTime();
        controller = new PIDController(P, I, D);
        profile = new AsymetricMotionProfile(startPos, finalPosition, constraints);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            currentPos = m.encoder.getPosition();
            profile.constraints.max_velocity = constraints.max_velocity;
            profile.constraints.max_acceleration = constraints.max_acceleration;
            profile.constraints.max_deceleration = constraints.max_deceleration;
            controller.setPID(P, I, D);
            MotionState state = profile.calculate(timer.time());
            double power = controller.calculate(currentPos, state.x);
            m.set(power);

            telemetry.addData("curPos", currentPos);
            telemetry.addData("tarPos", state.x);
            telemetry.addData("tarVel", state.v);
            telemetry.addData("tarAcc", state.a);
            telemetry.addData("startPos", startPos);
            telemetry.addData("finalPos", finalPosition);
            telemetry.update();
        }
    }
}
