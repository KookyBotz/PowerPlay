package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.geometry.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Constraints;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.State;

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
    private final DigitalChannel clawSensor;

    public AsymmetricMotionProfile profile;
    public State curState;
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
    public static double fourbar_down = 0.23;

    public static double pivot_flat = 0.43;
    public static double pivot_pitch_up = 0.37;
    public static double pivot_pitch_down = 0.47;
    public static double pivot_auto_transfer = 0.52;
    public static double pivot_pitch_score = 0.3;
    public static double pivot_pitch_pikcup = 0.25;

    public boolean isExtended = false;
    private boolean hasCone = false;

    public double pivotOffset = 0;

    public static final GrabPosition[] CYCLE_GRAB_POSITIONS = {
            new GrabPosition(550, 150, 0.457, pivot_pitch_up, 0),
            new GrabPosition(530, 150, 0.397, pivot_pitch_up, 0),
            new GrabPosition(520, 150, 0.345, pivot_pitch_up, 0),
            new GrabPosition(520, 150, 0.302, pivot_pitch_up, 0),
            new GrabPosition(520, 150, 0.242, pivot_pitch_up, 0)
    };

    private final double turret_deposit = 0;
    private final double turret_intake = 0.62;

    public double power = 0.0;
    public double targetPosition = 0.0;

    public int offset = 0;

    public PivotState pivotState = PivotState.FLAT;
    public FourbarState fourbarState = FourbarState.TRANSITION;

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
        UPRIGHT,
        DOWN
    }

    public enum PivotState {
        FLAT, PITCH_UP, SCORE, DOWN, PIVOT_AUTO_TRANSFER
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
        this.clawSensor = hardwareMap.get(DigitalChannel.class, "clawSensor");
        clawSensor.setMode(DigitalChannel.Mode.INPUT);

        this.profile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.controller = new PIDFController(P, I, D, F);
        this.timer = new ElapsedTime();
        this.voltageTimer = new ElapsedTime();
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
        timer.reset();
        voltageTimer.reset();
    }

    public void update(PivotState state) {
        pivotState = state;
        switch (state) {
            case FLAT:
                pivot.setPosition(pivot_flat + pivotOffset);
                break;
            case PITCH_UP:
                pivot.setPosition(pivot_pitch_up + pivotOffset);
                break;
            case SCORE:
                pivot.setPosition(pivot_pitch_score + pivotOffset);
                break;
            case DOWN:
                pivot.setPosition(pivot_pitch_pikcup + pivotOffset);
                break;
            case PIVOT_AUTO_TRANSFER:
                pivot.setPosition(pivot_auto_transfer + pivotOffset);
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
        fourbarState = state;
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
            case DOWN:
                setFourbar(fourbar_down);
                break;
        }
    }

    public void loop() {
        this.controller.setPIDF(P, I, D, F);
        hasCone = clawSensor.getState();

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.calculate(timer.time());
        if (curState.v != 0) {
            targetPosition = curState.x;
        }

        isExtended = (getPos() > 30);

        power = -controller.calculate(intakePosition, targetPosition) / voltage * 14;
    }

    public void read() {
        intakePosition = extensionEncoder.encoder.getPosition() + offset;
    }

    public void write() {
        extension.set(power);
    }

    public void setFourbar(double pos) {
        barLeft.setPosition(pos);
        barRight.setPosition(1 - pos);
    }

    public void setPivot(double pos){
        pivot.setPosition(pos);
    }

    public int getPos() {
        return (int) intakePosition;
    }

    public boolean hasCone() {
        return hasCone;
    }

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
        if (curState.v == 0 && newPosition >= -15 && newPosition <= 540) {
            targetPosition = newPosition;
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        this.newProfile(targetPos, max_v, max_a, max_a);
    }

    public void newProfile(double targetPos, double max_v, double max_a, double max_d) {
        profile = new AsymmetricMotionProfile(getPos(), targetPos, new Constraints(max_v, max_a, max_d));
        resetTimer();
    }

    public void adjustPivotOffset(double offset) {
        this.pivotOffset += offset;
    }
}
