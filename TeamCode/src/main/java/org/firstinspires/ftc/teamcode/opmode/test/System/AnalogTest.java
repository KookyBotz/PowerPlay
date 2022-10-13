package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class AnalogTest extends CommandOpMode {

    Servo claw;
    AnalogInput input;
    public static double position = 0.0;

    @Override
    public void initialize() {
        claw = hardwareMap.get(Servo.class, "claw");
        input = hardwareMap.get(AnalogInput.class, "claw");
    }

    @Override
    public void run() {
        claw.setPosition(position);

        telemetry.addData("Maximum Voltage", input.getMaxVoltage());
        telemetry.addData("Current Position", input.getVoltage() * 360 / 3.3);
        telemetry.addData("Current Voltage", input.getVoltage());
        telemetry.addData("Desired Position", position);
        telemetry.update();
    }
}
