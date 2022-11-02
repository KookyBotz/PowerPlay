package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonLynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.AsymmetricMotionProfile;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public final MotorEx extension;
    private final Servo barLeft, barRight;
    private final Servo claw, turret;

    public MotionProfile profile;
    public MotionConstraints constraints;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private double intakePosition;

    private double P = 0.05;
    private double I = 0.0;
    private double D = 0.0;

    public static int intake_out_pos = 400;

    private double claw_pos_open = 0.2;
    private double claw_pos_closed = 0.37;

    private double fourbar_extended = 0.07;
    private double fourbar_retracted = 0.75;
    private double fourbar_transition = fourbar_retracted - 0.2;

    private double turret_deposit = 0;
    private double turret_intake = 0.62;

    private final double FOURBAR_LENGTH = 9.842;

    public double power = 0.0;
    private double targetPosition = 0.0;

    // thanks aabhas <3
    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.extension = new MotorEx(hardwareMap, "extension");
        if (isAuto) {
            extension.resetEncoder();
        }
        this.barLeft = hardwareMap.get(Servo.class, "fourbarLeft");
        this.barRight = hardwareMap.get(Servo.class, "fourbarRight");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.turret = hardwareMap.get(Servo.class, "turret");

//        this.profile = new AsymmetricMotionProfile(0, 0, new MotionConstraints(0, 0, 0));
        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.timer = new ElapsedTime();
        timer.reset();
        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();
        this.controller = new PIDController(P, I, D);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop() {
        intakePosition = extension.encoder.getPosition();
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
//        if (curState.getV() != 0) {
//            targetPosition = curState.getX();
//        }

        power = controller.calculate(intakePosition, targetPosition) / voltage * 12;
        extension.set(power);


//        double target = profile.update(timer.time())[0];
//        if (distance < 0) {
//            target += startPosition;
//        }
//
//        power = controller.calculate(intakePosition, target) / voltage * 12;
//        extension.set(power);

        //AnalogInput sensor = new AnalogInput()
        //
        // AnalogInput claw = hardwareMap.get(AnalogInput.class, "clawName");
        //
        // mult by 360/33.33
    }

    public void setMotionProfile(AsymmetricMotionProfile profile) {
//        this.profile = profile;
        resetTimer();
    }

    public void setPos(int pos) {
        this.targetPosition = pos;
    }

    public void setFourbar(double pos) {
        barLeft.setPosition(pos);
        barRight.setPosition(1 - pos);
    }

    public void setClaw(double position) {
        claw.setPosition(position);
    }

    public int getPos() {
        return (int) intakePosition;
    }

    public int getTargetPos() {
        return (int) targetPosition;
    }

    public void extensionOut() {
        extension.setTargetPosition(intake_out_pos);
    }

    public void closeClaw() {
        claw.setPosition(claw_pos_closed);
    }

    public void openClaw() {
        claw.setPosition(claw_pos_open);
    }

    public void extendForebar() {
        barLeft.setPosition(fourbar_extended);
        barRight.setPosition(1 - fourbar_extended);
    }

    public void closeForebar() {
        barLeft.setPosition(fourbar_retracted);
        barRight.setPosition(1 - fourbar_retracted);
    }

    public void setFourbarFactor(double factor) {
        double fourbarAddition = 0.0035 * factor;
        double barLeftPos = barLeft.getPosition();
        if (!(barLeftPos + fourbarAddition > fourbar_retracted) || !(barLeftPos - fourbarAddition < fourbar_extended)) {
            barLeft.setPosition(barLeftPos + fourbarAddition);
            barRight.setPosition(1 - barLeftPos + fourbarAddition);
        }
    }

    public void transitionFourbar() {
        barLeft.setPosition(fourbar_transition);
        barRight.setPosition(1 - fourbar_transition);
    }

    public void intakeTurret() {
        turret.setPosition(turret_intake);
    }

    public void depositTurret() {
        turret.setPosition(turret_deposit);
    }

    public void setPower(double pow) {
        extension.set(pow);
    }

    public void resetTimer() {
        timer.reset();
    }

    public void setTargetPosition(double target){
        this.targetPosition = target;
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
    }
}
