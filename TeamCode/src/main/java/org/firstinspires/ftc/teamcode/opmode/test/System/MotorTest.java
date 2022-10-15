package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@TeleOp(name = "MotorConfigTest")
public class MotorTest extends OpMode {
    // port 0 reverse set to -1

    MotorEx a, b, c, d, e, f, g, h;
    @Override
    public void initialize() {
        a = new MotorEx(hardwareMap, "extension");
        a.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        b = new MotorEx(hardwareMap, "lift");
        c = new MotorEx(hardwareMap, "asd1");
        d = new MotorEx(hardwareMap, "asd2");
        e = new MotorEx(hardwareMap, "asd3");
        f = new MotorEx(hardwareMap, "asd4");
        g = new MotorEx(hardwareMap, "asd5");
        h = new MotorEx(hardwareMap, "asd6");
    }

    @Override
    public void run() {
//        if (gamepad1.a) {
//
//        }
        a.set(0.25);
        b.set(0.25);
        c.set(0.25);
        d.set(0.25);
        e.set(0.25);
        f.set(0.25);
        g.set(0.25);
        h.set(0.25);
    }
}
