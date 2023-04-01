package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.geometry.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Constraints;
import org.firstinspires.ftc.teamcode.common.drive.geometry.State;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private RobotHardware robot;

    // stores the state of the subsystem
    // anything other than GOOD means it goofed
    public STATE state = STATE.GOOD;
    public FourbarState fourbarState = FourbarState.INTERMEDIATE;
    public PivotState pivotState = PivotState.FLAT;
    public ClawState clawState = ClawState.OPEN;
    public TurretState turretState = TurretState.OUTWARDS;

    public AsymmetricMotionProfile intakeProfile;
    public AsymmetricMotionProfile fourbarProfile;
    public State intakeMotionState;
    public State fourbarMotionState;
    private final ElapsedTime intakeTimer;
    private final ElapsedTime fourbarTimer;
    private final ElapsedTime voltageTimer;
    public ElapsedTime intaketime;
    private PIDController controller;

    private double voltage = 0;
    private double intakePosition = 0;
    private double lastTargetPosition = 0;

    public static double P = 0.0121;
    public static double I = 0.0;
    public static double D = 0.00051;
    public static double F = 0.06;

    public static double INTAKE_DELAY = 0.02;

    public boolean isExtended = false;
    private boolean hasCone = false;
    private boolean notreached = false;
    private boolean done = false;
    private boolean withinTolerance = false;

    public double pivotOffset = 0;

//    public static final GrabPosition[] CYCLE_GRAB_POSITIONS = {
//            new GrabPosition(550, 150, 0.457, pivot_pitch_up, 0),
//            new GrabPosition(530, 150, 0.397, pivot_pitch_up, 0),
//            new GrabPosition(520, 150, 0.345, pivot_pitch_up, 0),
//            new GrabPosition(520, 150, 0.302, pivot_pitch_up, 0),
//            new GrabPosition(520, 150, 0.242, pivot_pitch_up, 0)
//    };

    private final double turret_deposit = 0;
    private final double turret_intake = 0.62;

    public double power = 0.0;
    public double targetPosition = 0.0;
    public double time = 0.0;


    public enum STATE {
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum ProfileTarget {
        INTAKE,
        FOURBAR
    }

    public enum TurretState {
        OUTWARDS,
        INTERMEDIATE,
        INWARDS
    }

    public enum ClawState {
        OPEN,
        CLOSED,
        AUTO
    }

    public enum FourbarState {
        INTAKE,
        LOW,
        GROUND,
        FALLEN,
        INTERMEDIATE,
        PRE_TRANSFER,
        TRANSFER
    }

//    public enum PivotState {
//        FLAT, PITCH_UP, SCORE, DOWN, PIVOT_AUTO_TRANSFER
//    }

    public enum PivotState {
        FLAT,
        LOW,
        FALLEN,
        TRANSFER,
        PRE_TRANSFER
    }

    // thanks aabhas <3
    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
        this.intakeProfile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.fourbarProfile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.intakeMotionState = new State(0, 0, 0);
        this.fourbarMotionState = new State(0, 0, 0);
        this.controller = new PIDController(P, I, D);
        this.intakeTimer = new ElapsedTime();
        this.fourbarTimer = new ElapsedTime();
        this.voltageTimer = new ElapsedTime();
        this.intaketime = new ElapsedTime();
        this.voltage = robot.voltageSensor.getVoltage();

        intakeTimer.reset();
        fourbarTimer.reset();
        voltageTimer.reset();
//        update(FourbarState.INTERMEDIATE);
        setFourbar(INTAKE_FOURBAR_INTERMEDIATE);
        update(TurretState.OUTWARDS);
        update(ClawState.OPEN);
        update(PivotState.FLAT);
    }

    public void update(PivotState state) {
        pivotState = state;
        switch (state) {
            case FLAT:
                robot.pivot.setPosition(INTAKE_PIVOT_FLAT + pivotOffset);
                break;
            case LOW:
                robot.pivot.setPosition(INTAKE_PIVOT_LOW + pivotOffset);
                break;
            case FALLEN:
                robot.pivot.setPosition(INTAKE_PIVOT_PICKUP + pivotOffset);
                break;
            case TRANSFER:
                robot.pivot.setPosition(INTAKE_PIVOT_TRANSFER + pivotOffset);
                break;
            case PRE_TRANSFER:
                robot.pivot.setPosition(INTAKE_PIVOT_PRE_TRANSFER + pivotOffset);
                break;
        }
    }

    public void update(TurretState state) {
        turretState = state;
        switch (state) {
            case OUTWARDS:
                robot.turret.setPosition(INTAKE_TURRET_OUTWARDS);
                break;
            case INTERMEDIATE:
                robot.turret.setPosition(INTAKE_TURRET_INTERMEDIATE);
                break;
            case INWARDS:
                robot.turret.setPosition(INTAKE_TURRET_INWARDS);
                break;
        }
    }

    public void update(ClawState state) {
        clawState = state;
        switch (state) {
            case OPEN:
                robot.claw.setPosition(INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                robot.claw.setPosition(INTAKE_CLAW_CLOSED);
                break;
            case AUTO:
                robot.claw.setPosition(INTAKE_CLAW_AUTO);
                break;
        }
    }

    public void update(FourbarState state) {
        fourbarState = state;
        switch(state) {
            case GROUND:
//                newProfile(INTAKE_FOURBAR_GROUND, new Constraints(0, 0, 0), ProfileTarget.FOURBAR);
                newProfile(INTAKE_FOURBAR_GROUND);
                break;
            case INTAKE:
//                newProfile(INTAKE_FOURBAR_INTAKE, new Constraints(0, 0, 0), ProfileTarget.FOURBAR);
//                setFourbar(INTAKE_FOURBAR_INTAKE);
                newProfile(INTAKE_FOURBAR_INTAKE);
                break;
            case LOW:
//                newProfile(INTAKE_FOURBAR_LOW, new Constraints(0, 0, 0), ProfileTarget.FOURBAR);
                newProfile(INTAKE_FOURBAR_LOW);
                break;
            case PRE_TRANSFER:
//                newProfile(INTAKE_FOURBAR_DEPOSIT, new Constraints(0, 0, 0), ProfileTarget.FOURBAR);
                newProfile(INTAKE_FOURBAR_PRE_TRANSFER);
                break;
            case TRANSFER:
                newProfile(INTAKE_FOURBAR_TRANSFER);
                break;
            case INTERMEDIATE:
//                newProfile(INTAKE_FOURBAR_INTERMEDIATE, new Constraints(0, 0, 0), ProfileTarget.FOURBAR);
                newProfile(INTAKE_FOURBAR_INTERMEDIATE);
                break;
        }
    }

//    public void update(FourbarState state) {
//        fourbarState = state;
//        switch (state) {
//            case INTAKE:
//                setFourbar(fourbar_extended);
//                break;
//            case TRANSITION:
//                setFourbar(fourbar_transition);
//                break;
//            case DEPOSIT:
//                setFourbar(fourbar_retracted);
//                break;
//            case SCORE:
//                setFourbar(fourbar_score);
//                break;
//            case UPRIGHT:
//                setFourbar(fourbar_upright);
//                break;
//            case DOWN:
//                setFourbar(fourbar_down);
//                break;
//        }
//    }

    public void loop() {
//        this.controller.setPID(P, I, D, F);
//
//        if (voltageTimer.seconds() > 5) {
//            voltage = robot.voltageSensor.getVoltage();
//            voltageTimer.reset();
        }

//        intakeMotionState = intakeProfile.calculate(intakeTimer.time());
//        if (intakeMotionState.v != 0) {
//            targetPosition = intakeMotionState.x;
//        }

//        fourbarMotionState = fourbarProfile.calculate(fourbarTimer.time());
//        if (fourbarMotionState.v != 0) {
//            targetFourbarPosition = fourbarMotionState.x;
//        }
//        setFourbar(targetFourbarPosition);

//        isExtended = getPos() > (INTAKE_EXTENDED_TOLERANCE * EXTENSION_TICKS_PER_INCH);

//        power = -controller.calculate(intakePosition, targetPosition) / voltage * 14;
//    }

    public void loop2 () {
        this.controller.setPID(P, I, D);
//        if (targetFourbarPosition != lastTargetFourbarPosition) {
//            newProfile(targetFourbarPosition);
//            lastTargetFourbarPosition = targetFourbarPosition;
//        }

        hasCone = !robot.clawSensor.getState();

        if (voltageTimer.seconds() > 5) {
            voltage = robot.voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        fourbarMotionState = fourbarProfile.calculate(fourbarTimer.time() + INTAKE_DELAY);
        if (fourbarMotionState.v != 0) {
            setFourbar(fourbarMotionState.x);
        }

        intakeMotionState = intakeProfile.calculate(intakeTimer.time());
        if (intakeMotionState.v != 0) {
            setTargetPosition((int) intakeMotionState.x);
        }

        withinTolerance = Math.abs(getPos() - getTargetPosition()) <= INTAKE_ERROR_TOLERANCE;

//        if (lastTargetPosition != publicTargetPosition) {
//            intaketime.reset();
//            done = false;
//        }
//
//        boolean reached = getPos() >= publicTargetPosition;
//        if (reached && !notreached && !done) {
//            time = intaketime.milliseconds();
//            intaketime.reset();
//            done = true;
//        }
//        notreached = reached;
        power = Range.clip((-controller.calculate(intakePosition, targetPosition) + (F * Math.signum(targetPosition - intakePosition)) / voltage * 14), -1, 1);
        if (targetPosition <= 0) {
            power -= -0.3;
        }
        // TODO: 0.066 is the compensate max
    }

    public void read() {
        try {
            intakePosition = robot.intakeEncoder.getPosition();
        } catch (Exception e) {
            intakePosition = 0;
        }

        try {
            hasCone = !robot.clawSensor.getState();
        } catch (Exception e) {
            hasCone = false;
        }
    }

    public void write() {
        try {
            robot.extension.set(power);
        } catch (Exception e) {}
    }

    public void setFourbar(double pos) {
        if (pos <= 0.066) {
            pos += 1.0399e-5 * (getPos());
        }
        robot.fourbarLeft.setPosition(pos);
        robot.fourbarRight.setPosition(1 - pos);
    }

    public void retractReset() {
        if (isWithinTolerance() && Math.abs(robot.intakeEncoder.getRawVelocity()) == 0 && getTargetPosition() <= 0) {
            robot.intakeEncoder.reset();
        }
    }

    public void setPivot(double pos){
        robot.pivot.setPosition(pos);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setFourbarTargetPosition(double targetPosition) {
        this.newProfile(targetPosition);
    }

    public int getPos() {
        return (int) intakePosition;
    }

    public int getTargetPosition() {
        return (int) targetPosition;
    }

    public double getPower() {
        return power;
    }

    public boolean hasCone() {
        return hasCone;
    }

    public boolean isWithinTolerance() { return withinTolerance; }

    public void setFourbarFactor(double factor) {
//        if (fourbarMotionState.v == 0) {
//            double fourbarAddition = INTAKE_FOURBAR_FACTOR * factor;
//            double fourbarPosition = robot.fourbarLeft.getPosition();
//
//            if (!(fourbarPosition + fourbarAddition > INTAKE_FOURBAR_DEPOSIT) || !(fourbarPosition - fourbarAddition < INTAKE_FOURBAR_INTAKE)) {
////                targetFourbarPosition = fourbarPosition + fourbarAddition;
//                setFourbar(fourbarPosition + fourbarAddition);
//                setFourbar(1 - fourbarPosition + fourbarAddition);
//            }
//        }
//        setFourbar(fourbarPosition + fourbarAddition);
//        setFourbar(1 - fourbarPosition + fourbarAddition);
        double fourbarAddition = INTAKE_FOURBAR_FACTOR * factor;
        double fourbarPosition = robot.fourbarLeft.getPosition();
        setFourbar(fourbarPosition + fourbarAddition);
    }

    public void setTurretFactor(double factor) {
        double turretAddition = INTAKE_TURRET_FACTOR * factor;
        double turretPos = robot.turret.getPosition();
        if (!(turretPos + turretAddition > turret_intake) || !(turretPos - turretAddition < turret_deposit)) {
            robot.turret.setPosition(turretPos + turretAddition);
        }
    }

    public void setSlideFactor(double factor) {
        double slideAddition = INTAKE_MANUAL_FACTOR * factor;
        double newPosition = intakePosition + slideAddition;
        if (intakeMotionState.v == 0 && newPosition >= INTAKE_MIN - 10 && newPosition <= INTAKE_MAX) {
            targetPosition = newPosition;
        }
    }

    public void newProfile(double targetPos, double max_v, double max_a, ProfileTarget target) {
        this.newProfile(targetPos, new Constraints(max_v, max_a, max_a), target);
    }

    public void newProfile(double targetPos, double max_v, double max_a, double max_d, ProfileTarget target) {
        this.newProfile(targetPos, new Constraints(max_v, max_a, max_d), target);
    }

    public void newProfile(double targetPos, Constraints constraints, ProfileTarget target) {
        constraints.convert(EXTENSION_TICKS_PER_INCH);
        switch (target) {
            case INTAKE:
                intakeProfile = new AsymmetricMotionProfile(getPos(), targetPos, constraints);
                intakeTimer.reset();
                break;
        }
    }

    public void newProfile(double targetPos) {
        Constraints constraints = new Constraints(INTAKE_FOURBAR_MAX_V, INTAKE_FOURBAR_MAX_A, (fourbarState.equals(FourbarState.INTERMEDIATE) ? INTAKE_FOURBAR_MAX_D_UP : INTAKE_FOURBAR_MAX_D_DOWN));
        fourbarProfile = new AsymmetricMotionProfile(robot.fourbarLeft.getPosition(), targetPos, constraints);
        fourbarTimer.reset();
    }

    public void adjustPivotOffset(double offset) {
        robot.pivot.setPosition(robot.pivot.getPosition() + offset);
        pivotOffset += offset;
    }
}
