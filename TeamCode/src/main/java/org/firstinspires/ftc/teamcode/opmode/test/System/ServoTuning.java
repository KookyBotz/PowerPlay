package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@Config
@TeleOp(name = "ServoTuning")
public class ServoTuning extends OpMode {
    Servo claw, turret, barLeft, barRight;

    public static double clawPos1, clawPos2;
    public static double turretPos1, turretPos2;

    public static double fourbarPos1, fourbarPos2;

    @Override
    public void initialize() {
        barLeft = hardwareMap.get(Servo.class, "fourbarLeft");
        barRight = hardwareMap.get(Servo.class, "fourbarRight");
        claw = hardwareMap.get(Servo.class, "claw");
        turret = hardwareMap.get(Servo.class, "turret");
    }

    @Override
    public void run() {
        if (gamepad1.a) {
            barLeft.setPosition(fourbarPos1);
            barRight.setPosition(1 - fourbarPos1);
        }

        if (gamepad1.b) {
            barLeft.setPosition(fourbarPos2);
            barRight.setPosition(1 - fourbarPos2);
        }

        if (gamepad1.x) {
            claw.setPosition(clawPos1);
        }

        if (gamepad1.y) {
            claw.setPosition(clawPos2);
        }

        if (gamepad1.right_bumper) {
            turret.setPosition(turretPos1);
        }

        if (gamepad1.left_bumper) {
            turret.setPosition(turretPos2);
        }
    }
}
