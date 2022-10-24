package org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MathUtils;


@Config
public class SwerveModule {
    public static PIDFCoefficients MODULE_PID = new PIDFCoefficients(0.95, 0, 0.02, 0);

    public static double K_STATIC = 0.03;

    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    //EXPERIMENTAL FEATURES
    public static boolean WAIT_FOR_TARGET = false;

    public static double ALLOWED_COS_ERROR = Math.toRadians(2);

    public static double ALLOWED_BB_ERROR = Math.toRadians(5);

    public static boolean MOTOR_FLIPPING = true;

    public static double FLIP_BIAS = Math.toRadians(0);

    public static double WHEEL_RADIUS = 1.4; // in
    public static double GEAR_RATIO = 1/(3.5*1.5*2); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;

    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;
    private PIDFController rotationController;

    private boolean wheelFlipped = false;



    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(1000, 2000));

        encoder = e;
        rotationController = new PIDFController(MODULE_PID.p, MODULE_PID.i, MODULE_PID.d, MODULE_PID.f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SwerveModule(HardwareMap hardwareMap, String mName, String sName, String eName) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(CRServo.class, sName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, eName)));
    }


    double realError = 0;
    public void update() {
        double target = getTargetRotation(), current = getModuleRotation();

        if(MOTOR_FLIPPING && Math.abs(AngleUnit.normalizeRadians(current-target))>Math.PI/2) {
            realError = target - current;
            current = AngleUnit.normalizeRadians(current + Math.PI);
            wheelFlipped = !wheelFlipped;
        }
        if (current - target > Math.PI) current -= (2 * Math.PI);
        else if (target - current > Math.PI) current += (2 * Math.PI);


        double power = Range.clip(rotationController.calculate(current), -MAX_SERVO, MAX_SERVO);
        if(Double.isNaN(power)) power = 0;

        servo.setPower(Math.abs(rotationController.getPositionError()) > ALLOWED_BB_ERROR ? power+K_STATIC*Math.signum(power) : 0);
    }

    public double getTargetRotation() {
        return rotationController.getSetPoint();
    }

    public double getModuleRotation() {
        return encoder.getCurrentPosition();
    }


    public int flipModifier(){
        return MOTOR_FLIPPING && wheelFlipped ? 1 : -1;
    }


    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(runMode, coefficients);
    }
    double lastMotorPower = 0;
    public void setMotorPower(double power) {
        //target check
        lastMotorPower = power;
        //flip check
        if(MOTOR_FLIPPING) power*=flipModifier();

        motor.setPower(power);
    }

    public boolean isWithinAllowedError(){
        double error = Math.abs(rotationController.getPositionError());
        return error < ALLOWED_COS_ERROR || error > 2*Math.PI - ALLOWED_COS_ERROR;
    }

    public void setServoPower(double power) {
        servo.setPower(power);
    }

    public static double MIN_MOTOR_TO_TURN = 0.05;
    public void setTargetRotation(double target) {
        if(Math.abs(lastMotorPower) < MIN_MOTOR_TO_TURN){
            //add stuff like X-ing preAlign
            return;
        }
        rotationController.setSetPoint(target);
    }

    public double getServoPower() {
        return servo.getPower();
    }

    public double getWheelPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }
    public SwerveModuleState asState(){
        return new SwerveModuleState(this);
    }

    public static class SwerveModuleState {
        public SwerveModule module;
        public double wheelPos, podRot;
        public SwerveModuleState(SwerveModule s){
            module = s;
            wheelPos = 0;
            podRot = 0;
        }

        public SwerveModuleState update(){
            return setState(-module.getWheelPosition(), module.getModuleRotation());
        }
        public SwerveModuleState setState(double wheel, double pod){
            wheelPos = wheel;
            podRot = pod;
            return this;
        }
        //TODO add averaging for podrots based off of past values
        public Vector2d calculateDelta(){
            double oldWheel = wheelPos;
            update();
            return Vector2d.polar(wheelPos-oldWheel, podRot);
        }
    }
    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}