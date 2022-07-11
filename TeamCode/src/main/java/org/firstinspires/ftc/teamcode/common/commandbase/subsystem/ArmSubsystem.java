package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;


@Config
public class ArmSubsystem extends SubsystemBase {
    public final DcMotorEx arm;
    private final Servo linkage;
    private final VoltageSensor batteryVoltageSensor;

    public static double linkage_in = 0.935;

    public static double linkage_out = 0.5;

    private final double p = 0.01;
    private final double d = 0.0005;
    private final double kcos = 0.2;
    private final double ticks_to_degrees = 700 / 180.0;

    private final PIDController controller;
    private ElapsedTime time;
    private ElapsedTime voltageTimer;
    private double voltage;

    private MotionProfile profile;
    public static double max_v = 10000;
    public static double max_a = 6000;

    private int target = 5;
    private int previous_target = 5;


    private double cache = 0;

    public ArmSubsystem(DcMotorEx a, Servo l, VoltageSensor b) {
        arm = a;
        linkage = l;

        controller = new PIDController(p, 0, d);
        controller.setPID(p, 0, d);
        this.batteryVoltageSensor = b;
        time = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();
    }

    public void loop() {
        if (target != previous_target) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(previous_target, 0), new MotionState(target, 0), max_v, max_a);
            time.reset();
            previous_target = target;
        }

        if (voltageTimer.seconds() > 5) {
            voltage = batteryVoltageSensor.getVoltage();
            voltageTimer.reset();
        }

        int armpos = arm.getCurrentPosition();
        cache = armpos;
        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double target = targetState.getX();
        double pid = controller.calculate(armpos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_to_degrees)) * kcos;

        double power = (pid + ff) / voltage * 12.0;

        arm.setPower(power);

        //dont ask
        linkage.setPosition(linkage.getPosition());
    }

    public void setPos(int pos) {
        target = pos;
    }

    public void armIn() {
        target = 5;
    }

    public void armShared() {
        target = 705;
    }

    public void linkageIn() {
        linkage.setPosition(linkage_in);
    }

    public int pos() {
        return arm.getCurrentPosition();
    }

    public void linkage(DoubleSupplier percentage) {
        linkage.setPosition(linkage_in + (linkage_out - linkage_in) * percentage.getAsDouble());
        System.out.println("linkage: " + percentage.getAsDouble());
    }

    public void adjustArm(DoubleSupplier percentage) {
        target = 705 + (int) (50 * percentage.getAsDouble());
    }

    public double getCachePos() {
        return cache;
    }
}
