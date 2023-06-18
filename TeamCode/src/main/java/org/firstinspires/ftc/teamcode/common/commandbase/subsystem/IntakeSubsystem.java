package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.geometry.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Constraints;
import org.firstinspires.ftc.teamcode.common.drive.geometry.State;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

import java.util.ArrayList;
import java.util.List;

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
    public ElapsedTime intaketime;
    private PIDController controller;

    private double intakePosition = 0;
    public int stackHeight = 4;

    public static double P = 0.0121;
    public static double I = 0.0;
    public static double D = 0.00051;
    public static double F = 0.06;

    public static double INTAKE_DELAY = 0.02;

    public final List<Boolean> coneDetected = new ArrayList<>();
    public final double[] stackHeights = {0.045, 0.08, 0.115, 0.14, 0.173};
    private boolean hasCone = false;
    private boolean withinTolerance = false;

    public static double pivotOffset = -0.05;

    private final double turret_deposit = 0;
    private final double turret_intake = 0.62;

    public double power = 0.0;
    public double targetPosition = 0.0;
    public double time = 0.0;
    public double intakeTime = 0.0;

    public boolean resetting = false;
    public boolean fallen = false;
    public boolean stackGrabbing = false;

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
        OPEN_AUTO,
        CLEAR
    }

    public enum FourbarState {
        INTAKE,
        LOW,
        GROUND,
        INTERMEDIATE,
        CLEAR,
        PRE_TRANSFER,
        TRANSFER,
        FALLEN
    }

    public enum PivotState {
        FLAT,
        FLAT_AUTO,
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
        this.intaketime = new ElapsedTime();

        intakeTimer.reset();
        fourbarTimer.reset();
        setFourbar(INTAKE_FOURBAR_INTERMEDIATE);
        update(TurretState.OUTWARDS);
        update(ClawState.OPEN);
        update(PivotState.FLAT);
    }

    public void update(PivotState state) {
        if (state != pivotState) {
            pivotOffset = -0.05;
        }
        pivotState = state;
        switch (state) {
            case FLAT_AUTO:
                robot.pivot.setPosition(INTAKE_PIVOT_FLAT_AUTO + pivotOffset);
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
            case OPEN_AUTO:
                robot.claw.setPosition(INTAKE_CLAW_OPEN_AUTO);
                break;
            case CLEAR:
                robot.claw.setPosition(INTAKE_CLAW_CLEAR);
                break;
        }
    }

    public void update(FourbarState state) {
        fourbarState = state;
        switch(state) {
            case GROUND:
                newProfile(INTAKE_FOURBAR_GROUND);
                break;
            case INTAKE:
                newProfile(INTAKE_FOURBAR_INTAKE);
                break;
            case LOW:
                newProfile(INTAKE_FOURBAR_LOW);
                break;
            case PRE_TRANSFER:
                newProfile(INTAKE_FOURBAR_PRE_TRANSFER);
                break;
            case TRANSFER:
                newProfile(INTAKE_FOURBAR_TRANSFER);
                break;
            case INTERMEDIATE:
                newProfile(INTAKE_FOURBAR_INTERMEDIATE);
                break;
            case FALLEN:
                newProfile(INTAKE_FOURBAR_FALLEN);
                break;
            case CLEAR:
                newProfile(INTAKE_FOURBAR_CLEAR_POS);
                break;
        }
    }

    public void loop2 () {
        this.controller.setPID(P, I, D);

        fourbarMotionState = fourbarProfile.calculate(fourbarTimer.time());
        if (fourbarMotionState.v != 0) {
            setFourbar(fourbarMotionState.x);
        }

        withinTolerance = Math.abs(getPos() - getTargetPosition()) <= INTAKE_ERROR_TOLERANCE;

        power = Range.clip((-controller.calculate(intakePosition, targetPosition) + (F * Math.signum(targetPosition - intakePosition)) / robot.getVoltage() * 14), -1, 1);
        if (targetPosition <= 0) {
            power -= -0.1;
        }

        if(resetting){
            power = 0.4;
        }
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
        if(robot.enabled) {
            try {
                robot.extension.set(power);
            } catch (Exception e) {
            }
        }else{
            robot.extension.set(0);
        }
    }

    public void retractIntakeExtension() {
        newIntakeProfile(0);
    }

    public void setFourbar(double pos) {
        robot.fourbarLeft.setPosition(pos - F_OFFSET);
        robot.fourbarRight.setPosition(1 - (pos + 0.005 - F_OFFSET));
    }

    public void retractReset() {
        robot.intakeEncoder.reset();
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
        if (intakeMotionState.v == 0 && newPosition >= INTAKE_MIN && newPosition <= INTAKE_MAX) {
            targetPosition = newPosition;
        }
    }

    public void changeStackHeight(int input) {
        stackHeight = Math.max(0, Math.min(4, stackHeight + input));
        newProfile(stackHeights[stackHeight]);
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

    public void newIntakeProfile(double targetPosition) {
        Constraints constraints = new Constraints(INTAKE_EXTENSION_MAX_V, INTAKE_EXTENSION_MAX_A, INTAKE_EXTENSION_MAX_D);
        intakeProfile = new AsymmetricMotionProfile(getPos(), targetPosition, constraints);
        intakeTimer.reset();
    }

    public void adjustPivotOffset(double offset) {
        robot.pivot.setPosition(robot.pivot.getPosition() + offset);
        pivotOffset += offset;
    }

    public double getFourbarPosition() {
        return robot.fourbarLeft.getPosition();
    }
}
