package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;

@Config
@TeleOp(name = "SlideTesting")
public class SlideTesting extends CommandOpMode {

    //private Robot robot;
    MotorEx extension;
    TrapezoidalMotionProfile profile;
    ElapsedTime timer;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;

    public static double maxD = 0;
    public static double maxV = 0.0;
    public static double maxA = 0.0;

    PIDController controller;

    boolean curState = true;
    boolean flag = true;

    @Override
    public void initialize() {
        //robot = new Robot(hardwareMap);
        extension = new MotorEx(hardwareMap, "extension");
        extension.resetEncoder();
        profile = new TrapezoidalMotionProfile(maxV, maxA, maxD);
        timer = new ElapsedTime();

        controller = new PIDController(P, I, D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        // extension.setTargetPosition((int) profile.update(timer.time())[0] * 28);
        //extension.setTargetPosition(pos);
        profile = new TrapezoidalMotionProfile(maxV, maxA, maxD);

        controller.setPID(P, I, D);
        double[] profiles = profile.update(timer.time());
        double pos2 = (int) profiles[0];
        double power = controller.calculate(extension.getCurrentPosition(), pos2);
        extension.set(power);
        if (extension.getCurrentPosition() >= maxD - 100 && curState) {
            curState = false;
            maxD = 0;
            timer.reset();
        }

        if (!curState && flag) {
            for (int i = 0; i < 2; i++) {
                profiles[i] *= -1;
            }
            flag = false;
        }
        telemetry.addData("power", power);
        telemetry.addData("curPo", extension.getCurrentPosition());
        telemetry.addData("motPo", profiles[0]);
        telemetry.addData("motVe", profiles[1]);
        telemetry.addData("motAc", profiles[2]);
        telemetry.addData("maxD", maxD);
        telemetry.addData("maxV", maxV);
        telemetry.addData("maxA", maxA);
        telemetry.addData("bools:", curState);
        telemetry.addData("boolsflag:", flag);
        telemetry.update();

        if (gamepad1.a) {
            timer.reset();
        }

        if (gamepad1.b) {
            extension.resetEncoder();
        }
    }
}
