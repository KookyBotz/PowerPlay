package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "SwerveTest")
public class SwerveTest extends OpMode {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public CRServo frontLeftServo;
    public CRServo frontRightServo;
    public CRServo backLeftServo;
    public CRServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLeftMotor.setPower(0.5);
            frontLeftServo.setPower(0.3);
        } else {
            frontLeftMotor.setPower(0);
            frontLeftServo.setPower(0);
        }

        if (gamepad1.b) {
            frontRightMotor.setPower(0.5);
            frontRightServo.setPower(0.3);
        } else {
            frontRightMotor.setPower(0);
            frontRightServo.setPower(0);
        }

        if (gamepad1.x) {
            backLeftMotor.setPower(0.5);
            backLeftServo.setPower(0.3);
        } else {
            backLeftMotor.setPower(0);
            backLeftServo.setPower(0);
        }

        if (gamepad1.y) {
            backRightMotor.setPower(0.5);
            backRightServo.setPower(0.3);
        } else {
            backRightMotor.setPower(0);
            backRightServo.setPower(0);
        }

        telemetry.addData("frontLeft", frontLeftEncoder.getVoltage());
    }
}
