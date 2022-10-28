package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "SwervePIDTest")
public class SwervePIDTest extends LinearOpMode {

    AnalogInput lfEncoder, lrEncoder, rrEncoder, rfEncoder;

    @Override
    public void runOpMode() throws InterruptedException {

        lfEncoder = hardwareMap.get(AnalogInput.class, "leftFrontEncoder");
        lrEncoder = hardwareMap.get(AnalogInput.class, "leftRearEncoder");
        rrEncoder = hardwareMap.get(AnalogInput.class, "rightRearEncoder");
        rfEncoder = hardwareMap.get(AnalogInput.class, "rightFrontEncoder");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("LF", lfEncoder.getVoltage());
            telemetry.addData("LR", lrEncoder.getVoltage());
            telemetry.addData("RR", rrEncoder.getVoltage());
            telemetry.addData("RF", rfEncoder.getVoltage());
            telemetry.update();
        }
    }
}
