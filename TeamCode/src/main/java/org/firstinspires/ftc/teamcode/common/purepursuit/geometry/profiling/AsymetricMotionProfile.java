package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionState;

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

        this.dt1 = Math.abs(constraints.max_velocity) / Math.abs(constraints.max_acceleration);
        this.dt3 = Math.abs(constraints.max_velocity) / Math.abs(constraints.max_deceleration);

        double averageDt = (this.dt1 + this.dt3) / 2;
        this.dt2 = Math.abs(distance) / Math.abs(constraints.max_velocity) - averageDt;


    }

    public MotionState calculate(double seconds) {
        return null;
    }
}
