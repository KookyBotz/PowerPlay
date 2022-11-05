package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;
@Disabled

@TeleOp(name = "WiringTest")
public class WiringTest extends OpMode {

    Servo a, b, c, d;

    @Override
    public void initialize() {
        a = hardwareMap.get(Servo.class, "claw");
        b = hardwareMap.get(Servo.class, "turret");
        c = hardwareMap.get(Servo.class, "fourbarLeft");
        d = hardwareMap.get(Servo.class, "fourbarRight");
    }

    @Override
    public void run() {
//        if (gamepad1.a) {
//            c.setPosition(0.5);
//            d.setPosition(0.5);
//        }
//
//        // left
//        if (gamepad1.x) {
//            c.setPosition(0);
//            d.setPosition(1);
//        }
//
//        // right
//        if (gamepad1.y) {
//            d.setPosition(1);
//        }

    }
}
