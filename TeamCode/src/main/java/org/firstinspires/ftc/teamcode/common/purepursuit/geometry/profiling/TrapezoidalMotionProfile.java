package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class TrapezoidalMotionProfile implements MotionProfile {
    private final double maxV;
    private final double maxA;
    private final double distance;

    public TrapezoidalMotionProfile(double maxV, double maxA, double distance) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.distance = distance;
    }

    @Override
    public double update(double time)
    {
        double velocity = 0.0;
        return Math.min(velocity, maxV);
    }
}
