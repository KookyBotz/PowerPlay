package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "ServoModeSwerveTest")
public class ServoModeSwerveTest extends OpMode {
    CRServo a, b, c, d;
    @Override
    public void initialize() {
        a = hardwareMap.get(CRServo.class, "rightFrontServo");
        b = hardwareMap.get(CRServo.class, "rightRearServo");
        c = hardwareMap.get(CRServo.class, "leftFrontServo");
        d = hardwareMap.get(CRServo.class, "leftRearServo");


        ((CRServoImplEx) a).setPwmRange(new PwmControl.PwmRange(515, 2485));
        ((CRServoImplEx) b).setPwmRange(new PwmControl.PwmRange(515, 2485));
        ((CRServoImplEx) c).setPwmRange(new PwmControl.PwmRange(515, 2485));
        ((CRServoImplEx) d).setPwmRange(new PwmControl.PwmRange(515, 2485));
    }

    @Override
    public void run() {
        a.setPower(gamepad1.right_stick_y);
        b.setPower(gamepad1.right_stick_y);
        c.setPower(gamepad1.right_stick_y);
        d.setPower(gamepad1.right_stick_y);
    }

}
