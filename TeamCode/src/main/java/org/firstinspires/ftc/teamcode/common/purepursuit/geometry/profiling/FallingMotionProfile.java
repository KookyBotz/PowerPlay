package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class FallingMotionProfile implements MotionProfile {
    private final double maxV;
    private final double maxA;

    public FallingMotionProfile(double maxV, double maxA) {
        this.maxV = maxV;
        this.maxA = maxA;
    }

    @Override
    public double[] update(double time) {
        return new double[]{0, 0, 0};
    }
}
