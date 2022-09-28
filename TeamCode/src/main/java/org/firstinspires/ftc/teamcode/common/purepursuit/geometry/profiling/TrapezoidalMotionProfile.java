package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class TrapezoidalMotionProfile implements MotionProfile {
    private final double maxV;
    private final double maxA;

    public TrapezoidalMotionProfile(double maxV, double maxA) {
        this.maxV = maxV;
        this.maxA = maxA;
    }

    @Override
    public double update(double time) {
        return 0;
    }
}
