package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@Config
@TeleOp(name = "AnalogTest")
public class AnalogTest extends CommandOpMode {

    Servo claw;
    AnalogInput input;
    public static double position = 0.0;

    @Override
    public void initialize() {
        claw = hardwareMap.get(Servo.class, "claw");
        input = hardwareMap.get(AnalogInput.class, "claw");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
