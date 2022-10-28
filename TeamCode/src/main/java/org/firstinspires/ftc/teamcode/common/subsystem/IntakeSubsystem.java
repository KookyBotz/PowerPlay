package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;

import javax.xml.transform.TransformerException;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx extension;
    private final Servo barLeft, barRight;
    private final Servo claw, turret;

    private MotionProfile profile;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private double intakePosition;

    private double P = 0.02;
    private double I = 0.0;
    private double D = 0.0;

    private double distance = 0.0;
    private double maxV = 0.0;
    private double maxA = 0.0;

    public static int intake_out_pos = 400;

    public static double claw_pos_open = 0.2;
    public static double claw_pos_closed = 0.36;

    public static double fourbar_extended = 0.075;
    public static double fourbar_transition = 0.495;
    public static double fourbar_retracted = 0.75;

    private double turret_deposit = 0;
    private double turret_intake = 0.62;

    public static final double FOURBAR_LENGTH = 9.842;

    public double power = 0.0;
    public double startPosition = 0.0;

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

        this.profile = new TrapezoidalMotionProfile(maxV, maxA, distance);
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

        double target = profile.update(timer.time())[0];
        if (distance < 0) {
            target += startPosition;
        }
        power = controller.calculate(intakePosition, target) / voltage * 12;
        extension.set(power);

        //AnalogInput sensor = new AnalogInput()
                //
                // AnalogInput claw = hardwareMap.get(AnalogInput.class, "clawName");
                //
                // mult by 360/33.33
    }

    public void setMotionProfile(MotionProfile profile) {
        this.profile = profile;
    }

    public void setPos(int pos) {
        this.distance = pos;
        resetTimer();
    }

    public void setFourbar(double pos) {
        barLeft.setPosition(pos);
        barRight.setPosition(1 - pos);
    }

    public void setClaw(double position) {
        claw.setPosition(position);
    }

    public int getPos() {
        return extension.encoder.getPosition();
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

    public void intakeTurret() {
        turret.setPosition(turret_intake);
    }

    public void depositTurret() {
        turret.setPosition(turret_deposit);
    }

    public void resetTimer() {
        timer.reset();
    }

    public void setPID(double P, double I, double D) {
        controller.setPID(P, I, D);
    }

    public void setDVA(double d, double v, double a) {
        this.distance = d;
        this.maxV = v;
        this.maxA = a;
        this.profile = new TrapezoidalMotionProfile(maxV, maxA, distance);
        if (d < 0) {
            this.startPosition = Math.abs(d);
        } else {
            this.startPosition = 0;
        }
    }
}
