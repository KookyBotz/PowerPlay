package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class RisingMotionProfile implements MotionProfile {
    private final double maxV;
    private final double maxA;

    public RisingMotionProfile(double maxV, double maxA) {
        this.maxV = maxV;
        this.maxA = maxA;
    }

    @Override
    public double[] update(double time) {
        if (time < 0) {
            return null;
        }
        double velocity = (maxA * time) / maxV;
        return new double[]{0, Math.min(velocity, maxV), 0};
    }
}