package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionState;

public class AsymmetricMotionProfile {
    // TODO put initialposition -> constraints on final when done
    public double initialPosition;
    public double finalPosition;
    public MotionConstraints constraints;

    protected double dt1;
    protected double dt2;
    protected double dt3;

    protected double profileDuration;
    protected double distance;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, MotionConstraints constraints) {
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

        if (this.dt2 < 0) {
            this.dt2 = 0;
            if (Math.abs(this.constraints.max_acceleration) > Math.abs(constraints.max_deceleration)) {
                constraints.max_acceleration = Math.abs(Math.abs(constraints.max_deceleration));
            }
            else {
                constraints.max_deceleration = Math.abs(Math.abs(constraints.max_acceleration));
            }
            this.dt1 = Math.sqrt(Math.abs(distance)/Math.abs(constraints.max_acceleration));
            this.dt3 = Math.sqrt(Math.abs(distance)/Math.abs(constraints.max_deceleration));
        }
        this.profileDuration = this.dt1 + this.dt2 + this.dt3;

    }

    public MotionState calculate(double seconds) {
        double acceleration;
        double velocity;
        double position;

        if (seconds <= this.dt1) {
            acceleration = Math.abs(constraints.max_acceleration);
            velocity = seconds * acceleration;
            position = 0.5 * acceleration * Math.pow(seconds, 2);
        } else if (seconds <= this.dt1 + this.dt2) {
            acceleration = 0;
            velocity = Math.abs(calculate(this.dt1).v);

            position = calculate(this.dt1).x + constraints.max_velocity * (seconds - this.dt1);
        } else if (seconds <= this.dt1 + this.dt2 + this.dt3) {
            acceleration = Math.abs(constraints.max_deceleration);
            double coastVelocity = Math.abs(calculate(this.dt1).v);
            velocity = coastVelocity - (seconds - this.dt1 - this.dt2) * acceleration;

            double endofdt2 = this.dt1 + this.dt2;
            double endOfdt2Pos = Math.abs(calculate(endofdt2).x);
            position = endOfdt2Pos + coastVelocity * (seconds - endofdt2) - 0.5 * acceleration * Math.pow(seconds - endofdt2, 2);

            acceleration *= -1;

        } else {
            acceleration = 0;
            velocity = 0;
            return new MotionState(finalPosition,
                    velocity * Math.signum(distance), acceleration * Math.signum(distance));
        }

        return new MotionState(this.initialPosition + position * Math.signum(distance),
                velocity * Math.signum(distance), acceleration * Math.signum(distance));
    }
}
