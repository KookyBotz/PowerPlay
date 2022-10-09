package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class TrapezoidalMotionProfile implements MotionProfile {
    private final double maxV;
    private final double maxA;
    private final double distance;
    private final double tNorm;
    private final double tA;

    public TrapezoidalMotionProfile(double maxV, double maxA, double distance) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.distance = distance;

        this.tNorm = distance / maxV;
        this.tA = maxV / maxA;
    }

    @Override
    public double update(double time)
    {
        i
    }
}
