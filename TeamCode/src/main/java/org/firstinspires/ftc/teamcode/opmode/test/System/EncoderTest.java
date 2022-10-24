package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends OpMode {
    AnalogInput mod1, mod2, mod3, mod4;

    @Override
    public void initialize() {
        mod1 = hardwareMap.get(AnalogInput.class, "mod1");
        mod2 = hardwareMap.get(AnalogInput.class, "mod2");
        mod3 = hardwareMap.get(AnalogInput.class, "mod3");
        mod4 = hardwareMap.get(AnalogInput.class, "mod4");
    }

    @Override
    public void run() {
        telemetry.addData("mod1volt", mod1.getVoltage());
        telemetry.addData("mod2volt", mod2.getVoltage());
        telemetry.addData("mod3volt", mod3.getVoltage());
        telemetry.addData("mod4volt", mod4.getVoltage());
        telemetry.update();
    }
}
