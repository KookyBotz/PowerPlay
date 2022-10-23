package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SwervePIDTest extends LinearOpMode {

    private DcMotorEx motor;
    private CRServo servo;
    public static PIDFCoefficients COEFFICIENTS = new PIDFCoefficients(0, 0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        servo = hardwareMap.get(CRServo.class, "leftFrontServo");

    }
}
