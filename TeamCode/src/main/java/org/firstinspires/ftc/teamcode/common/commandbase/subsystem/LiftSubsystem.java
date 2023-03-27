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

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

@Config
public class LiftSubsystem extends SubsystemBase {

    // stores the state of the subsystem
    // anything other than GOOD means it goofed
    private RobotHardware robot;
    public LiftSubsystem.STATE state = LiftSubsystem.STATE.GOOD;
    public LiftState liftState = LiftState.RETRACTED;
    public LatchState latchState = LatchState.LATCHED;

    private AsymmetricMotionProfile liftProfile;
    public State liftMotionState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private PIDController controller;

    private double voltage;
    private int liftPosition;
    private double power = 0.0;
    private int targetPosition = 0;

    private boolean isExtended = false;
    private boolean hasCone = false;
    private boolean withinTolerance = false;

    public static double P = 0.01;
    public static double I = 0.1;
    public static double D = 0.000125;
    public static double F = 0.1;

    public enum STATE {
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum LatchState {
        LATCHED,
        UNLATCHED,
        INTERMEDIATE
    }

    public enum LiftState {
        HIGH,
        MID,
        RETRACTED
    }

    // thanks aabhas <3
    public LiftSubsystem(RobotHardware robot) {
        this.robot = robot;

        this.liftProfile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.controller = new PIDController(P, I, D);
        this.voltage = robot.voltageSensor.getVoltage();
        this.voltageTimer = new ElapsedTime();
        this.timer = new ElapsedTime();

        if (AUTO) {
            update(LatchState.LATCHED);
        } else {
            update(LatchState.UNLATCHED);
        }
    }

    public void update(LatchState state) {
        latchState = state;
        switch(state) {
            case LATCHED:
                robot.latch.setPosition(LIFT_LATCHED);
                break;
            case UNLATCHED:
                robot.latch.setPosition(LIFT_UNLATCHED);
                break;
            case INTERMEDIATE:
                robot.latch.setPosition(LIFT_INTERMEDIATE);
                break;
        }
    }

    public void update(LiftState state) {
        liftState = state;
        switch (state) {
            case HIGH:
                setTargetPos(LIFT_HIGH_POS);
//                newProfile(LIFT_HIGH_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
            case MID:
                setTargetPos(LIFT_MID_POS);
//                newProfile(LIFT_MID_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
            case RETRACTED:
                setTargetPos(LIFT_RETRACT_POS);
//                newProfile(LIFT_RETRACT_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
        }
    }

    public void loop() {
        this.controller.setPID(P, I, D);

        if (voltageTimer.seconds() > 5) {
            voltage = robot.voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        liftMotionState = liftProfile.calculate(timer.time());
        if (liftMotionState.v != 0) {
            setTargetPos((int) liftMotionState.x);
        }

        withinTolerance = Math.abs(getPos() - getTargetPos()) < LIFT_ERROR_TOLERANCE;

//        isExtended = getPos() > (LIFT_EXTENDED_TOLERANCE * LIFT_TICKS_PER_INCH);
//        hasCone = !robot.depositSensor.getState();

        power = Range.clip(((-controller.calculate(liftPosition, targetPosition) + (F * Math.signum(targetPosition - liftPosition))) / voltage * 14), -1, 1);
    }

    public void read() {
        try {
            liftPosition = robot.liftEncoder.getPosition();
        } catch (Exception e) {
            liftPosition = 0;
        }
    }

    public void write() {
        try {
            robot.liftLeft.set(power);
            robot.liftRight.set(-power);
        } catch (Exception e) {}
    }

    public double getPos() {
        return liftPosition;
    }

    public void setTargetPos(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPos() {
        return targetPosition;
    }

    public int getStatePos(LiftState state) {
        switch(state) {
            case HIGH:
                return LIFT_HIGH_POS;
            case MID:
                return LIFT_MID_POS;
            case RETRACTED:
                return LIFT_RETRACT_POS;
        }
        return 0;
    }

    public double getPower() {
        return power;
    }

    public boolean hasCone() { return hasCone; }

    public boolean isWithinTolerance() { return withinTolerance; }

    public void setSlideFactor(double factor) {
        double slideAddition = LIFT_MANUAL_FACTOR * factor;
        double newPosition = liftPosition + slideAddition;
        if (liftMotionState.v == 0 && newPosition >= 0 && newPosition <= LIFT_MAX) {
            setTargetPos((int) newPosition);
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, Constraints constraints) {
        constraints.convert(LIFT_TICKS_PER_INCH);
        this.liftProfile = new AsymmetricMotionProfile(getPos(), targetPos, constraints);
        resetTimer();
    }
}
