package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final MotorEx lift;

    private MotionProfile profile;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;

    private double P = 0.02;
    private double I = 0.0;
    private double D = 0.0;

    private double distance = 0.0;
    private double maxV = 20;
    private double maxA = 20;

    public static int high_pos = 100;
    public static int medium_pos = 75;
    public static int low_pos = 50;
    public static int retracted = 0;

    public double power = 0.0;

    // thanks aabhas <3
    public LiftSubsystem(HardwareMap hardwareMap) {
        this.lift = new MotorEx(hardwareMap, "lift");
        lift.resetEncoder();
        lift.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.profile = new TrapezoidalMotionProfile(maxV, maxA, 0);
        this.timer = new ElapsedTime();
        timer.reset();
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

        profile = new TrapezoidalMotionProfile(maxV, maxA, distance);
        controller.setPID(P, I, D);
        double target = profile.update(timer.time())[0];
        power = controller.calculate(lift.encoder.getPosition(), target) / voltage * 12;
        lift.set(power);

    }

    public void setPos(int pos) {
        lift.setTargetPosition(pos);
    }

    public int getPos() {
        return lift.encoder.getPosition();
    }

    // TODO reset timer internally in setpos
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
    }
}
