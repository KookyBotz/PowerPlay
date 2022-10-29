package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;

public class AsymetricMotionProfile {
    public final double initialPosition;
    public final double finalPosition;
    public final MotionConstraints constraints;

    protected double dt1;
    protected double dt2;
    protected double dt3;

    protected double profileDuration;
    protected double distance;

    public AsymetricMotionProfile(double initialPosition, double finalPosition, MotionConstraints constraints) {
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.constraints = constraints;
        compute();
    }

    protected void compute() {
        distance = finalPosition - initialPosition;


    }
}
