package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "SwerveWheelTest")
public class SwerveWheelTest extends OpMode {
    Servo a, b, c, d;
    DcMotorEx mA, mB, mC, mD;

    @Override
    public void initialize() {
        a = hardwareMap.get(Servo.class, "a");
        b = hardwareMap.get(Servo.class, "b");
        c = hardwareMap.get(Servo.class, "c");
        d = hardwareMap.get(Servo.class, "d");

        mA = hardwareMap.get(DcMotorEx.class, "mA");
        mB = hardwareMap.get(DcMotorEx.class, "mB");
        mC = hardwareMap.get(DcMotorEx.class, "mC");
        mD = hardwareMap.get(DcMotorEx.class, "mD");
    }

    @Override
    public void run() {
        if (gamepad1.a) {
            a.setPosition(1);
        } else {
            a.setPosition(0);
        }

        if (gamepad1.b) {
            b.setPosition(1);
        } else {
            b.setPosition(0);
        }

        if (gamepad1.x) {
            c.setPosition(1);
        } else {
            c.setPosition(0);
        }

        if (gamepad1.y) {
            d.setPosition(1);
        } else {
            d.setPosition(0);
        }

        if (gamepad1.right_bumper) {
            mA.setPower(1.0);
            mB.setPower(1.0);
            mC.setPower(1.0);
            mD.setPower(1.0);
        }

        if (gamepad1.left_bumper) {
            mA.setPower(0);
            mB.setPower(0);
            mC.setPower(0);
            mD.setPower(0);
        }

        if (gamepad1.dpad_down) {
            mA.setPower(1.0);
        } else {
            mA.setPower(0);
        }

        if (gamepad1.dpad_left) {
            mB.setPower(1.0);
        } else {
            mB.setPower(0);
        }

        if (gamepad1.dpad_up) {
            mC.setPower(1.0);
        } else {
            mC.setPower(0);
        }

        if (gamepad1.dpad_right) {
            mD.setPower(1.0);
        } else {
            mD.setPower(0);
        }
    }

}
