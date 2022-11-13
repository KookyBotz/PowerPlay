package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public final MotorEx extension;
    private final Servo barLeft, barRight;
    private final Servo claw, turret;

    public MotionProfile profile;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private double intakePosition;

    private final double P = 0.025;
    private final double I = 0.0;
    private final double D = 0.0;

    private final double claw_pos_open = 0.2;
    private final double claw_pos_closed = 0.37;

    private final double fourbar_extended = 0.15;
    private final double fourbar_retracted = 0.83;
    public final double fourbar_transition = fourbar_retracted - 0.2;
    private final double[] fourbar_pickup_position = new double[]{
            .15,
            .17,
            .22,
            .24,
            .32
    };

    private final double turret_deposit = 0;
    private final double turret_intake = 0.62;

    public double power = 0.0;
    public double targetPosition = 0.0;

    private TurretState turretState;
    private ClawState clawState;
    private FourbarState fourbarState;

    public enum TurretState {
        INTAKE,
        MANUAL,
        DEPOSIT
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public enum FourbarState {
        INTAKE,
        MANUAL,
        TRANSITION,
        DEPOSIT
    }

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

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), 30, 25);

        this.timer = new ElapsedTime();
        timer.reset();
        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();
        this.controller = new PIDController(P, I, D);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void update(TurretState state) {
        switch(state) {
            case INTAKE:
                turret.setPosition(turret_intake);
            case DEPOSIT:
                turret.setPosition(turret_deposit);
        }

        turretState = state;
    }

    public void update(ClawState state) {
        switch(state) {
            case OPEN:
                claw.setPosition(claw_pos_open);
            case CLOSED:
                claw.setPosition(claw_pos_closed);
        }

        clawState = state;
    }

    public void update(FourbarState state) {
        switch (state) {
            case INTAKE:
                barLeft.setPosition(fourbar_extended);
                barRight.setPosition(1 - fourbar_extended);
            case TRANSITION:
                barLeft.setPosition(fourbar_transition);
                barRight.setPosition(1 - fourbar_transition);
            case DEPOSIT:
                barLeft.setPosition(fourbar_retracted);
                barRight.setPosition(1 - fourbar_retracted);
        }

        fourbarState = state;
    }

    public void loop() {
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
        }

        power = controller.calculate(intakePosition, targetPosition) / voltage * 12;

        // TODO add analog input claw gang
        //AnalogInput sensor = new AnalogInput()
        //
        // AnalogInput claw = hardwareMap.get(AnalogInput.class, "clawName");
        //
        // multiply by 360/33.33
    }

    // TODO optimize read/writes for every 1/2 or 1/4 loop
    public void read() {
        intakePosition = extension.encoder.getPosition();
    }

    public void write() {
        extension.set(power);
    }

    public void setPos(int pos) {
        this.targetPosition = pos;
    }

    // TODO get rid of all current setfourbar command/positions to use just update (FSM)
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

    public double getFourbarPos() {
        return barLeft.getPosition();
    }

    public void closeClaw() {
        claw.setPosition(claw_pos_closed);
    }

    public void openClaw() {
        claw.setPosition(claw_pos_open);
    }

    public void extendFourbar() {
        barLeft.setPosition(fourbar_extended);
        barRight.setPosition(1 - fourbar_extended);
    }

    public void extendFourbar(int index) {
        barLeft.setPosition(fourbar_pickup_position[index]);
        barRight.setPosition(1 - fourbar_pickup_position[index]);
    }

    public void closeFourbar() {
        barLeft.setPosition(fourbar_retracted);
        barRight.setPosition(1 - fourbar_retracted);
    }

    public void transitionFourbar() {
        barLeft.setPosition(fourbar_transition);
        barRight.setPosition(1 - fourbar_transition);
    }

    public void setFourbarFactor(double factor) {
        double fourbarAddition = -0.007 * factor;
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
        double slideAddition = 10 * factor;
        double newPosition = intakePosition + slideAddition;
        if (curState.getV() == 0 && newPosition >= -15 && newPosition <= 485) {
            targetPosition = newPosition;
        }

    }

    public void intakeTurret() {
        turret.setPosition(turret_intake);
    }

    public void depositTurret() {
        turret.setPosition(turret_deposit);
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new com.acmerobotics.roadrunner.profile.MotionState(getPos(), 0), new com.acmerobotics.roadrunner.profile.MotionState(targetPos, 0), max_v, max_a);
        resetTimer();
    }
}
