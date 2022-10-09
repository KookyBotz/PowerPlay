package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.config.Config;
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

    public static int pos = 0;

    PIDController controller;

    @Override
    public void initialize() {
        //robot = new Robot(hardwareMap);
        extension = new MotorEx(hardwareMap, "extension");
        profile = new TrapezoidalMotionProfile(10, 20, 10);
        timer = new ElapsedTime();

        controller = new PIDController(P, I, D);
    }

    @Override
    public void run() {
        // extension.setTargetPosition((int) profile.update(timer.time())[0] * 28);
        extension.setTargetPosition(pos);
        controller.setPID(P, I, D);
        double power = controller.calculate();
        extension.set(power);
    }
}
