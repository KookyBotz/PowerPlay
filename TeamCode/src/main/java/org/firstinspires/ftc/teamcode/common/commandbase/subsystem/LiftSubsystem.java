package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.geometry.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Constraints;
import org.firstinspires.ftc.teamcode.common.drive.geometry.State;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

@Config
public class LiftSubsystem extends SubsystemBase {

    // stores the state of the subsystem
    // anything other than GOOD means it goofed
    public LiftSubsystem.STATE state = LiftSubsystem.STATE.GOOD;
    public LiftState liftState = LiftState.RETRACTED;
    public LatchState latchState = LatchState.LATCHED;

    private AsymmetricMotionProfile profile;
    public State curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private PIDFController controller;

    private double voltage;
    private int liftPosition;
    private double power = 0.0;
    private int targetPosition = 0;

    private boolean isExtended = false;
    private boolean hasCone = false;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    public enum STATE {
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum LatchState {
        LATCHED,
        UNLATCHED
    }

    public enum LiftState {
        HIGH,
        MID,
        RETRACTED
    }

    // thanks aabhas <3
    public LiftSubsystem() {

//        if (AUTO) {
//            update(LatchState.LATCHED);
//        } else {
//            update(LatchState.UNLATCHED);
//        }

        this.profile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.controller = new PIDFController(P, I, D, F);
        this.voltage = robot.voltageSensor.getVoltage();
        this.voltageTimer = new ElapsedTime();
        this.timer = new ElapsedTime();
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
        }
    }

    public void update(LiftState state) {
        liftState = state;
        switch (state) {
            case HIGH:
                newProfile(LIFT_HIGH_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
            case MID:
                newProfile(LIFT_MID_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
            case RETRACTED:
                newProfile(LIFT_RETRACT_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
        }
    }

    public void loop() {
        this.controller.setPIDF(P, I, D, F);

        if (voltageTimer.seconds() > 5) {
            voltage = robot.voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.calculate(timer.time());
        if (curState.v != 0) {
            targetPosition = (int) curState.x;
        }

        isExtended = getPos() > (LIFT_EXTENDED_TOLERANCE * LIFT_TICKS_PER_INCH);
//        hasCone = !robot.depositSensor.getState();

        power = (-controller.calculate(liftPosition, targetPosition) / voltage * 14);
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
            robot.liftRight.set(power);
        } catch (Exception e) {
            //
        }

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

    public boolean hasCone() { return hasCone; }

    public void setSlideFactor(double factor) {
        double slideAddition = LIFT_MANUAL_FACTOR * factor;
        double newPosition = liftPosition + slideAddition;
        if (curState.v == 0 && newPosition >= 0 && newPosition <= LIFT_MAX) {
            setTargetPos((int) newPosition);
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        this.newProfile(targetPos, new Constraints(max_v, max_a, max_a));
    }

    public void newProfile(double targetPos, double max_v, double max_a, double max_d) {
        this.newProfile(targetPos, new Constraints(max_v, max_a, max_d));
    }

    public void newProfile(double targetPos, Constraints constraints) {
        constraints.convert(LIFT_TICKS_PER_INCH);
        this.profile = new AsymmetricMotionProfile(getPos(), targetPos, constraints);
        resetTimer();
    }
}
