package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Encoder;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final MotorEx lift;
    public final MotorEx liftEncoder;
    public final Servo latch;

    private MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private int liftPosition;

    private final double P = 0.0375;
    private final double I = 0.0;
    private final double D = 0.0;

    public double power = 0.0;
    public double targetPosition = 0.0;

    private boolean moving = false;

    private LiftState liftState;
    private LatchState latchState;

    public enum LiftState {
        HIGH,
        MIDDLE,
        LOW,
        RETRACTED
    }

    public enum LatchState {
        LATCHED,
        UNLATCHED
    }

    // thanks aabhas <3
    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.lift = new MotorEx(hardwareMap, "lift");
        // TODO find the exact config name troll
        this.liftEncoder = new MotorEx(hardwareMap, "leftRearMotor");
        this.latch = hardwareMap.get(Servo.class, "latch");
        this.timer = new ElapsedTime();
        timer.reset();

        if (isAuto) {
            liftEncoder.resetEncoder();
            update(LatchState.LATCHED);
        } else {
            update(LatchState.UNLATCHED);
        }
        liftEncoder.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        update(LiftState.RETRACTED);

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop() {
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
            moving = false;
        } else {
            moving = true;
        }

        power = controller.calculate(liftPosition, targetPosition) / voltage * 12;
        power = Math.max(Math.min(power, 0.6), -0.6);
    }

    public void update(LiftState state) {
        switch(state) {
            case RETRACTED:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(0, 0), 3500, 7500);
                resetTimer();
                break;
            case LOW:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(150, 0), 400, 1500);
                resetTimer();
                break;
            case MIDDLE:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(385, 0), 400, 1500);
                resetTimer();
                break;
            case HIGH:
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(610, 0), 700, 3000);
                resetTimer();
                break;
        }

        liftState = state;
    }

    public void update(LatchState state) {
        switch(state) {
            case LATCHED:
                latch.setPosition(0.85);
                break;
            case UNLATCHED:
                latch.setPosition(0.27);
                break;
        }

        latchState = state;
    }

    public void read() {
        liftPosition = liftEncoder.encoder.getPosition();
    }

    public void write() {
        lift.set(power);
    }

    public int getPos() {
        return liftPosition;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void setSlideFactor(double factor) {
        double slideAddition = 10 * factor;
        double newPosition = liftPosition + slideAddition;
        if (curState.getV() == 0 && newPosition >= -15 && newPosition <= 405) {
            targetPosition = newPosition;
        }
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getPos(), 0), new MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }
}
