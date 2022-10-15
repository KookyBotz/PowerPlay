package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "SwerveWheelTest")
public class SwerveWheelTest extends OpMode {
    Servo a, b, c, d;

    @Override
    public void initialize() {
        a = hardwareMap.get(Servo.class, "a");
        b = hardwareMap.get(Servo.class, "b");
        c = hardwareMap.get(Servo.class, "c");
        d = hardwareMap.get(Servo.class, "d");
    }

    @Override
    public void run() {
        if (gamepad1.a) {
            a.setPosition(1);
        }

        if (gamepad1.b) {
            b.setPosition(1);
        }

        if (gamepad1.x) {
            c.setPosition(1);
        }

        if (gamepad1.y) {
            d.setPosition(1);
        }

    }

}
