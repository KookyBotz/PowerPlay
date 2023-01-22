package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;

@Config
public class IntakeSubsystem extends SubsystemBase {

    // stores the state of the subsystem
    // anything other than GOOD means it goofed
    public STATE state = STATE.GOOD;

    public final MotorEx extension;
    public final MotorEx extensionEncoder;
    private final Servo barLeft, barRight;
    private final Servo claw, turret;
    private final Servo pivot;
//    private final Rev2mDistanceSensorEx distanceSensor;

    public MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private PIDFController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private double intakePosition;

    public static double P = 0.06;
    public static double I = 0.013;
    public static double D = 0.0;
    public static double F = 0.0001;

    public static double claw_pos_open = 0.3;
    public static double claw_pos_closed = 0.48;

    public static double fourbar_extended = 0.23;
    public static double fourbar_retracted = 0.952;
    public static double fourbar_transition = fourbar_retracted - 0.15;
    public static double fourbar_score = 0.71;
    public static double fourbar_upright = 0.3;

    public static double pivot_flat = 0.43;
    public static double pivot_pitch_up = 0.37;
    public static double pivot_pitch_down = 0.47;
    public static double pivot_auto_transfer = 0.52;
    public static double pivot_pitch_score = 0.3;

    public double offset2 = 0;

    public static final GrabPosition[] CYCLE_GRAB_POSITIONS = {
            new GrabPosition(560, 150, 0.457, pivot_pitch_up, 0),
            new GrabPosition(540, 150, 0.402, pivot_pitch_up, 0),
            new GrabPosition(530, 150, 0.35, pivot_pitch_up, 0),
            new GrabPosition(530, 150, 0.307, pivot_pitch_up, 0),
            new GrabPosition(530, 150, 0.252 , pivot_pitch_up, 0)
    };

    private final double turret_deposit = 0;
    private final double turret_intake = 0.62;

    public double power = 0.0;
    public double targetPosition = 0.0;

    public int offset = 0;

    public enum STATE {
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum TurretState {
        INTAKE,
        DEPOSIT
    }

    public enum ClawState {
        OPEN,
        CLOSED,
    }

    public enum FourbarState {
        INTAKE,
        TRANSITION,
        DEPOSIT,
        SCORE,
        UPRIGHT
    }

    public enum PivotState {
        FLAT, PITCH_UP, SCORE
    }

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.extension = new MotorEx(hardwareMap, "extension");
        this.extensionEncoder = new MotorEx(hardwareMap, "rightRearMotor");
        if (isAuto) {
            extensionEncoder.resetEncoder();
        }
        this.barLeft = hardwareMap.get(Servo.class, "fourbarLeft");
        this.barRight = hardwareMap.get(Servo.class, "fourbarRight");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.turret = hardwareMap.get(Servo.class, "turret");
        this.pivot = hardwareMap.get(Servo.class, "pivot");

//        Rev2mDistanceSensor ds = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
//        this.distanceSensor = new Rev2mDistanceSensorEx(ds.getDeviceClient());
//        this.distanceSensor.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);


        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.timer = new ElapsedTime();
        timer.reset();
        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();
        this.controller = new PIDFController(P, I, D, F);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void update(PivotState state) {
        switch (state) {
            case FLAT:
                pivot.setPosition(pivot_flat + offset2);
                break;
            case PITCH_UP:
                pivot.setPosition(pivot_pitch_up + offset2);
                break;
            case SCORE:
                pivot.setPosition(pivot_pitch_score + offset2);
                break;
        }
    }

    public void update(TurretState state) {
        switch (state) {
            case INTAKE:
                turret.setPosition(turret_intake);
                break;
            case DEPOSIT:
                turret.setPosition(turret_deposit);
                break;
        }
    }

    public void update(ClawState state) {
        switch (state) {
            case OPEN:
                claw.setPosition(claw_pos_open);
                break;
            case CLOSED:
                claw.setPosition(claw_pos_closed);
                break;
        }
    }

    public void update(FourbarState state) {
        switch (state) {
            case INTAKE:
                setFourbar(fourbar_extended);
                break;
            case TRANSITION:
                setFourbar(fourbar_transition);
                break;
            case DEPOSIT:
                setFourbar(fourbar_retracted);
                break;
            case SCORE:
                setFourbar(fourbar_score);
                break;
            case UPRIGHT:
                setFourbar(fourbar_upright);
                break;
        }
    }

    public void loop() {
        controller = new PIDFController(P, I, D, F);

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
        }

        power = -controller.calculate(intakePosition, targetPosition) / voltage * 12;
    }

    public void read() {
        intakePosition = extensionEncoder.encoder.getPosition() + offset;
    }

    public void write() {
        extension.set(power);
    }

    public void setFourbar(double pos) {
        barLeft.setPosition(pos);
        barRight.setPosition(1 - pos + 0.05); //fight backlash
    }

    public void setPivot(double pos){
        pivot.setPosition(pos);
    }

    public int getPos() {
        return (int) intakePosition;
    }

//    public boolean hasCone() {
//        return distanceSensor.getDistance(DistanceUnit.CM) < distanceThreshold;
//    }

    public void setFourbarFactor(double factor) {
        double fourbarAddition = -0.01 * factor;
        double barLeftPos = barLeft.getPosition();
        if (!(barLeftPos + fourbarAddition > fourbar_retracted) || !(barLeftPos - fourbarAddition < fourbar_extended)) {
            barLeft.setPosition(barLeftPos + fourbarAddition);
            barRight.setPosition(1 - barLeftPos + fourbarAddition);
        }
    }

    public void setTurretFactor(double factor) {
        double turretAddition = 0.007 * factor;
        double turretPos = turret.getPosition();
        if (!(turretPos + turretAddition > turret_intake) || !(turretPos - turretAddition < turret_deposit)) {
            turret.setPosition(turretPos + turretAddition);
        }
    }

    public void setSlideFactor(double factor) {
        double slideAddition = 13 * factor;
        double newPosition = intakePosition + slideAddition;
        if (curState.getV() == 0 && newPosition >= -15 && newPosition <= 540) {
            targetPosition = newPosition;
        }
    }


    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }

    public void adjustPivotOffset(double offset) {
        this.offset2 += offset;
    }
}
