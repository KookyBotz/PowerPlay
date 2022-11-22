package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends OpMode {
    DcMotor encoder;

    @Override
    public void initialize() {
        encoder = hardwareMap.get(DcMotor.class, "encoder");
    }

    @Override
    public void run() {
        telemetry.addData("mod1volt", encoder.getCurrentPosition());
        telemetry.update();
    }
}
