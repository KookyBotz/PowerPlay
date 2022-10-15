package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

public class MotorTest extends OpMode {
    // port 0 reverse set to -1

    MotorEx a, b, c, d, e, f, g, h;
    @Override
    public void initialize() {
        a = new MotorEx(hardwareMap, "extension");
        a.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        b = new MotorEx(hardwareMap, "asd");
        c = new MotorEx(hardwareMap, "asd1");
        d = new MotorEx(hardwareMap, "asd2");
        e = new MotorEx(hardwareMap, "asd3");
        f = new MotorEx(hardwareMap, "asd4");
        g = new MotorEx(hardwareMap, "asd5");
        h = new MotorEx(hardwareMap, "asd6");
    }

    @Override
    public void run() {
        a.set(-0.1);
        b.set(0.1);
        c.set(0.1);
        d.set(0.1);
        e.set(0.1);
        f.set(0.1);
        g.set(0.1);
        h.set(0.1);
    }
}
