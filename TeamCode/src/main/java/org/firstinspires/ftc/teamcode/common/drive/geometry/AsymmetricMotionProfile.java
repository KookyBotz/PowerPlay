package org.firstinspires.ftc.teamcode.common.drive.geometry;

// currently redoing some of this math

public class AsymmetricMotionProfile {
    private final double initialPosition;
    private final double finalPosition;
    private final ProfileConstraints constraints;

    private double distance;
    private double t1;
    private double t2;
    private double t3;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, ProfileConstraints constraints) {
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.constraints = constraints;

        this.distance = finalPosition - initialPosition;

        this.t1 = constraints.max_velocity / constraints.acceleration;
        this.t3 = constraints.max_velocity / constraints.deceleration;

        double temp = Math.abs(finalPosition - initialPosition) - ((t1 + t3) / 2);
        this.t2 = 0;

        if (temp >= 0) {
           this.t2 = temp;
        }
    }

    public ProfileState calculate(double time) {
        double acceleration;
        double velocity;
        double position;

        if (time <= t1) {
            acceleration = this.constraints.acceleration;
            velocity = time * this.constraints.acceleration;
            position = (this.constraints.acceleration * Math.pow(time, 2)) / 2;
        } else if (time <= t1 + t2) {
            acceleration = 0;
            velocity = this.constraints.max_velocity;
            position = Math.abs((this.constraints.acceleration * Math.pow(this.t1, 2)) / 2) + this.constraints.max_velocity * (time - this.t1);
        } else if (time <= t1 + t2 + t3) {
            acceleration = this.constraints.deceleration;
            velocity = this.constraints.max_velocity - (time - this.t1 - this.t2) * this.constraints.deceleration;
            position = Math.abs((this.constraints.acceleration * Math.pow(this.t1, 2)) / 2) + this.constraints.max_velocity * (time - this.t1) - (this.constraints.deceleration * Math.pow((time - this.t1 - this.t2), 2)) / 2;
            acceleration *= -1;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        return new ProfileState((finalPosition < initialPosition) ? initialPosition - position : initialPosition + position, velocity, acceleration);
    }
}