package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final DcMotorEx lift;

    private MotionProfile profile;
    private final ElapsedTime timer;
    private final PIDController controller;

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
    public double[] funcs;

    // thanks aabhas <3
    public LiftSubsystem(HardwareMap hardwareMap) {
        this.lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        this.controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);
        this.profile = new TrapezoidalMotionProfile(maxV, maxA, 0);

        this.timer = new ElapsedTime();
    }

    public void loop() {
        profile = new TrapezoidalMotionProfile(maxV, maxA, distance);
        controller.setPID(P, I, D);
        funcs = profile.update(timer.time());
        power = controller.calculate(lift.getCurrentPosition(), funcs[0]);
        lift.setPower(power);
    }

    public void setPos(int pos) {
        lift.setTargetPosition(pos);
    }

    public int getPos() {
        return lift.getCurrentPosition();
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
    }
}
