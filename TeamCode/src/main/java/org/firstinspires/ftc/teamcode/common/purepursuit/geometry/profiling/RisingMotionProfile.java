package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class RisingMotionProfile implements MotionProfile {
    private final double maxVelocity;
    private final double maxAcceleration;

    public RisingMotionProfile(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    @Override
    public double update(double time) {
        if (time < 0) {
            return 0;
        }
        double velocity = (maxAcceleration * time) / maxVelocity;
        return Math.min(velocity, maxVelocity);
    }
}
