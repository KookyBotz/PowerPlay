package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public interface MotionProfile {
    final double maxAcceleration;
    final double maxVelocity;
//
//    public MotionProfile(double maxAcceleration, double maxVelocity) {
//        this.maxAcceleration = maxAcceleration;
//        this.maxVelocity = maxVelocity;
//    }
//
//    public double update(double time) {
//        if (time < 0) {
//            return 0;
//        }
//        double velocity = (maxAcceleration * time) / maxVelocity;
//        double thing = Math.min(velocity, maxVelocity);
//        System.out.println(thing);
//        return thing;
//    }
}
