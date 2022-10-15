package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@TeleOp(name = "WiringTest")
public class WiringTest extends OpMode {

    Servo a, b, c, d;

    @Override
    public void initialize() {
        a = hardwareMap.get(Servo.class, "port0");
        b = hardwareMap.get(Servo.class, "port1");
        c = hardwareMap.get(Servo.class, "port2");
        d = hardwareMap.get(Servo.class, "port3");
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
