package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
            frontLeftMotor.setPower(1);
//            frontLeftServo.setPower(1);
        } else {
            frontLeftMotor.setPower(0);
            frontLeftServo.setPower(0);
        }

        if (gamepad1.b) {
            frontRightMotor.setPower(1);
//            frontRightServo.setPower(1);
        } else {
            frontRightMotor.setPower(0);
            frontRightServo.setPower(0);
        }

        if (gamepad1.x) {
            backLeftMotor.setPower(1);
//            backLeftServo.setPower(1);
        } else {
            backLeftMotor.setPower(0);
            backLeftServo.setPower(0);
        }

        if (gamepad1.y) {
            backRightMotor.setPower(1);
//            backRightServo.setPower(1);
        } else {
            backRightMotor.setPower(0);
            backRightServo.setPower(0);
        }

//        telemetry.addData("frontLeft", frontLeftEncoder.getVoltage());
//        telemetry.addData("frontRight", frontRightEncoder.getVoltage());
//        telemetry.addData("backRight", backRightEncoder.getVoltage());
//        telemetry.addData("backLeft", backLeftEncoder.getVoltage());
        telemetry.addData("frontLeftCurrent", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("frontRightCurrent", frontRightMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("backRightCurrent", backRightMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("backLeftCurrent", backLeftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
