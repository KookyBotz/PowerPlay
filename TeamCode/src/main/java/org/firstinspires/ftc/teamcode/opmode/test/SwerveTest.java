package org.firstinspires.ftc.teamcode.opmode.test;

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
        frontRightServo =  hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo =  hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo =hardwareMap.get(CRServo.class, "backRightServo");

//        ( (CRServoImplEx) frontLeftCRServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        ( (CRServoImplEx) frontRightCRServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        ( (CRServoImplEx) backLeftCRServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        ( (CRServoImplEx) backRightCRServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLeftMotor.setPower(1);
//            frontLeftCRServoImplEx.setPosition(1);
        } else {
            frontLeftMotor.setPower(0);
//            frontLeftCRServoImplEx.setPosition(0.5);
        }

        if (gamepad1.b) {
            frontRightMotor.setPower(1);
//            frontRightCRServoImplEx.setPosition(1);
        } else {
            frontRightMotor.setPower(0);
//            frontRightCRServoImplEx.setPosition(0.5);
        }

        if (gamepad1.x) {
            backLeftMotor.setPower(1);
//            backLeftCRServoImplEx.setPosition(1);
        } else {
            backLeftMotor.setPower(0);
//            backLeftCRServoImplEx.setPosition(0.5);
        }

        if (gamepad1.y) {
            backRightMotor.setPower(1);
//            backRightCRServoImplEx.setPosition(1);
        } else {
            backRightMotor.setPower(0);
//            backRightCRServoImplEx.setPosition(0.5);
        }

        telemetry.addData("frontLeft", frontLeftEncoder.getVoltage());
        telemetry.addData("frontRight", frontRightEncoder.getVoltage());
        telemetry.addData("backRight", backRightEncoder.getVoltage());
        telemetry.addData("backLeft", backLeftEncoder.getVoltage());
        telemetry.addData("frontLeftCurrent", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("frontRightCurrent", frontRightMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("backRightCurrent", backRightMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("backLeftCurrent", backLeftMotor.getCurrent(CurrentUnit.AMPS));


//        telemetry.addData("frontLeft", frontLeftEncoder.getVoltage());
//        telemetry.addData("frontRight", frontRightEncoder.getVoltage());
//        telemetry.addData("backRight", backRightEncoder.getVoltage());
//        telemetry.addData("backLeft", backLeftEncoder.getVoltage());
        telemetry.update();
    }
}
