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

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionState;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final MotorEx lift;

    private AsymmetricMotionProfile profile;
    public MotionConstraints constraints;
    public MotionState curState;
    private final ElapsedTime timer;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;

    private double voltage;
    private double liftPosition;

    private double P = 0.025;
    private double I = 0.0;
    private double D = 0.0;

    private double distance = 0.0;
    private double maxV = 0.0;
    private double maxA = 0.0;

    public static int high_pos = 500;
    public static int high_maxV = 1500;
    public static int high_maxA = 7500;

    public static int medium_pos = 75;
    public static int low_pos = 50;
    public static int retracted = 0;

    public double power = 0.0;
    public double targetPosition = 0.0;

    // thanks aabhas <3
    public LiftSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.lift = new MotorEx(hardwareMap, "lift");
        if (isAuto) {
            lift.resetEncoder();
        }
        lift.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.profile = new AsymmetricMotionProfile(maxV, maxA, new MotionConstraints(0, 0, 0));
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
        liftPosition = lift.encoder.getPosition();
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

//        double target = profile.update(timer.time())[0];
//        if (distance < 0) {
//            target += startPosition;
//        }
//        power = controller.calculate(liftPosition, target) / voltage * 12;
//        lift.set(power);
        curState = profile.calculate(timer.time());
        if (curState.v != 0) {
            targetPosition = curState.x;
        }

        power = controller.calculate(liftPosition, targetPosition) / voltage * 12;
        lift.set(power);
    }

    public void setPos(int pos) {
        lift.setTargetPosition(pos);
        resetTimer();
    }

    public int getPos() {
        return lift.encoder.getPosition();
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
//        this.profile = new TrapezoidalMotionProfile(maxV, maxA, distance);
//        if (d < 0) {
//            this.startPosition = Math.abs(d);
//        } else {
//            this.startPosition = 0;
//        }
    }

    public void setMotionProfile(AsymmetricMotionProfile profile) {
        this.profile = profile;
        resetTimer();
    }
}
