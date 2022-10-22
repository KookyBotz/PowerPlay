package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@Config
@TeleOp(name = "ProfilingTest")
public class ProfilingTest extends CommandOpMode {

    MotorEx m;
    TrapezoidalMotionProfile profile;
    PIDController controller;
    ElapsedTime timer;

    public static double maxV = 0.0, maxA = 0.0, distance = 0.0;
    public static double P = 0.0, I = 0.0, D = 0.0;

    private double currentPos = 0.0;

    @Override
    public void initialize() {
        m = new MotorEx(hardwareMap, "motor1");
        m.resetEncoder();
        currentPos = m.encoder.getPosition();
        profile = new TrapezoidalMotionProfile(maxV, maxA, distance);
        timer = new ElapsedTime();
        controller = new PIDController(P, I, D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        currentPos = m.encoder.getPosition();
        profile.recondition(maxV, maxA, distance);
        controller.setPID(P, I, D);
        double[] profiles = profile.update(timer.time());
        double power = controller.calculate(currentPos, distance);
        m.set(power);

        telemetry.addData("power", power);
        telemetry.addData("currentPosition", currentPos);
        telemetry.addData("projectedPosition", profiles[0]);
        telemetry.addData("projectedVelocity", profiles[1]);
        telemetry.addData("projectedAcceleration", profiles[2]);
        telemetry.update();

        if (gamepad1.a) {
            timer.reset();
            m.resetEncoder();
        }

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
}
