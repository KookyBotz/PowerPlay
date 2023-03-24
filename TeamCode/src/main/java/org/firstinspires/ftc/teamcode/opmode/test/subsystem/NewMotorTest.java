package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "goofy ahh motor test")
@Config
public class NewMotorTest extends OpMode {

    public static double power = 0.1;

    public MotorEx motorIntake;
    public MotorEx motorLiftLeft;
    public MotorEx motorLiftRight;
    public Motor.Encoder motorIntakeEncoder;
    public Motor.Encoder motorLiftEncoder;

    @Override
    public void init() {
//        motorIntake = new MotorEx(hardwareMap, "intake");
//        motorIntakeEncoder = new MotorEx(hardwareMap, "frontRightMotor").encoder;
//        motorIntakeEncoder.reset();
        motorLiftLeft = new MotorEx(hardwareMap, "motorLiftLeft");
        motorLiftEncoder = new MotorEx(hardwareMap, "frontLeftMotor").encoder;
        motorLiftEncoder.reset();
        motorLiftRight = new MotorEx(hardwareMap, "motorLiftRight");
    }

    @Override
    public void loop() {
//        if (gamepad1.a) {
//            motorIntake.set(power);
//        } else {
//            motorIntake.set(0);
//        }

        if (gamepad1.b) {
            motorLiftLeft.set(power);
        } else {
            motorLiftLeft.set(0);
        }

        if (gamepad1.y) {
            motorLiftRight.set(-power);
        } else {
            motorLiftRight.set(0);
        }

//        telemetry.addData("frontRightMotor + intakeEncoder", motorIntakeEncoder.getPosition());
        telemetry.addData("frontLeftMotor + motorLiftEncoder", motorLiftEncoder.getPosition());

        telemetry.update();
    }
}
