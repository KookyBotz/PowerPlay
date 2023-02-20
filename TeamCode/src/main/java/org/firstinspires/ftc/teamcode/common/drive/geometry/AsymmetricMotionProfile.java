package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class AsymmetricMotionProfile {
    public final double initialPosition;
    public final double finalPosition;
    public final double distance;
    public final double totalTime;
    public final double initialVelocity;
    public final ProfileConstraints constraints;

    public double t1;
    public double t2;
    public double t3;

    public double vA = 0;
    public double xA = 0;

    private final boolean flipped;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, final ProfileConstraints constraints) {
        flipped = initialPosition > finalPosition;
        if (flipped) {
            double initPos = initialPosition;
            initialPosition = finalPosition;
            finalPosition = initPos;
        }

        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.distance = finalPosition - initialPosition;
        this.initialVelocity = 0;
        this.constraints = constraints;

        this.t1 = constraints.max_velocity / constraints.max_acceleration;
        this.t3 = constraints.max_velocity / Math.abs(constraints.max_deceleration);

        this.t2 = (Math.abs(distance) / constraints.max_velocity) - (t1 + t3) / 2;
        if (t2 < 0) {
            this.t2 = 0.0;

            double a = (constraints.max_acceleration / 2) * (1 - (constraints.max_acceleration / constraints.max_deceleration));
            double c = -finalPosition;

            this.t1 = (Math.sqrt(-4 * a * c)) / (2 * a);
            this.t3 = -(constraints.max_acceleration * t1) / constraints.max_deceleration;
        }

        xA = initialVelocity + initialVelocity * t1 + (constraints.max_acceleration * Math.pow(t1, 2)) / 2;
        vA = initialVelocity + constraints.max_acceleration * t1;

        // constraints.max_velocity = (distance < 0) ? -constraints.max_velocity : constraints.max_velocity;

        this.totalTime = t1 + t2 +t3;
    }

    public ProfileState calculate(double time) {
        double position;
        double velocity;
        double acceleration;

        if (time <= t1) {
            acceleration = constraints.max_acceleration;
            velocity = time * acceleration;
            position = initialPosition * initialVelocity + (constraints.max_acceleration * Math.pow(time, 2)) / 2;
        } else if (time <= t1 + t2) {
            acceleration = 0;
            velocity = constraints.max_velocity;
            position = xA + constraints.max_velocity * (time - t1);
        } else if (time <= t1 + t2 + t3) {
            acceleration = constraints.max_deceleration;
            velocity = vA - ((time - t1 - t2) * Math.abs(constraints.max_deceleration));
            position = xA + t2 * vA - (constraints.max_deceleration * Math.pow(time - t1 - t2, 2)) / 2;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        return new ProfileState((flipped) ? finalPosition - position : position, velocity, acceleration);
    }
}