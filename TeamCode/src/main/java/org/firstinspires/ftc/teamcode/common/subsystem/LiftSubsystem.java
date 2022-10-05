package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final DcMotorEx lift;

    private final PIDController controller;
    private final MotionProfile profile;

    private ElapsedTime timer;

    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    public static double maxV = 2;
    public static double maxA = 1;

    public static int high_pos = 100;
    public static int medium_pos = 75;
    public static int low_pos = 50;
    public static int retracted = 0;

    public LiftSubsystem(DcMotorEx lift) {
        this.lift = lift;

        this.controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.profile = new TrapezoidalMotionProfile(maxV, maxA, 0);

        this.timer = new ElapsedTime();
    }

    public void loop() {
        // not finished, still need to add pid control to this
        double power = profile.update(timer.time());
        lift.setPower(power);
    }

    public void setPos(int pos) {
        lift.setTargetPosition(pos);
    }

    public int getPos() {
        return lift.getCurrentPosition();
    }

    public void resetTime() {
        timer.reset();
    }
}
