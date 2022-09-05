package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

public class MotionProfile {
    private final double maxAcceleration;
    private final double maxVelocity;

    public MotionProfile(double maxAcceleration, double maxVelocity) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
    }

    public double update(double time) {
        if (time < 0) {
            return 0;
        }
        double velocity = (maxAcceleration * time) / maxVelocity;

        return Math.min(velocity, maxVelocity);
    }
}
