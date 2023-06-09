package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "SwerveTest")
public class SwerveTest extends OpMode {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public Servo frontLeftServoImplEx;
    public Servo frontRightServoImplEx;
    public Servo backLeftServoImplEx;
    public Servo backRightServoImplEx;

//    public AnalogInput frontLeftEncoder;
//    public AnalogInput frontRightEncoder;
//    public AnalogInput backLeftEncoder;
//    public AnalogInput backRightEncoder;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftServoImplEx = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRightServoImplEx =  hardwareMap.get(Servo.class, "frontRightServo");
        backLeftServoImplEx =  hardwareMap.get(Servo.class, "backLeftServo");
        backRightServoImplEx =hardwareMap.get(Servo.class, "backRightServo");

//        ( (ServoImplEx) frontLeftServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        ( (ServoImplEx) frontRightServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        ( (ServoImplEx) backLeftServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        ( (ServoImplEx) backRightServoImplEx).setPwmRange(new PwmControl.PwmRange(500, 2500));

//        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
//        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
//        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
//        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLeftMotor.setPower(1);
            frontLeftServoImplEx.setPosition(1);
        } else {
            frontLeftMotor.setPower(0);
            frontLeftServoImplEx.setPosition(0.5);
        }

        if (gamepad1.b) {
            frontRightMotor.setPower(1);
            frontRightServoImplEx.setPosition(1);
        } else {
            frontRightMotor.setPower(0);
            frontRightServoImplEx.setPosition(0.5);
        }

        if (gamepad1.x) {
            backLeftMotor.setPower(1);
            backLeftServoImplEx.setPosition(1);
        } else {
            backLeftMotor.setPower(0);
            backLeftServoImplEx.setPosition(0.5);
        }

        if (gamepad1.y) {
            backRightMotor.setPower(1);
            backRightServoImplEx.setPosition(1);
        } else {
            backRightMotor.setPower(0);
            backRightServoImplEx.setPosition(0.5);
        }

//        telemetry.addData("frontLeft", frontLeftEncoder.getVoltage());
//        telemetry.addData("frontRight", frontRightEncoder.getVoltage());
//        telemetry.addData("backRight", backRightEncoder.getVoltage());
//        telemetry.addData("backLeft", backLeftEncoder.getVoltage());
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
