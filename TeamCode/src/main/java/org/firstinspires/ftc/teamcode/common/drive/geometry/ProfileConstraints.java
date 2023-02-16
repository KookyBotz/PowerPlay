package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class ProfileConstraints {
    protected double max_velocity;
    public double acceleration;
    public double deceleration;

    public ProfileConstraints(double max_velocity, double acceleration, double deceleration) {
        this.max_velocity = Math.abs(max_velocity);
        this.acceleration = (acceleration <= 1e-7) ? Double.MAX_VALUE : Math.abs(acceleration);
        this.deceleration = (deceleration <= 1e-7) ? Double.MAX_VALUE : Math.abs(deceleration);
    }
}