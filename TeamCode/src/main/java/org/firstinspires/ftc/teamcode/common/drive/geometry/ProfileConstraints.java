package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class ProfileConstraints {
    public double max_velocity;
    public double max_acceleration;
    public double max_deceleration;

    public ProfileConstraints(double max_velocity, double acceleration, double deceleration) {
        this.max_velocity = Math.abs(max_velocity);
        this.max_acceleration = (acceleration == 0) ? 9999999999999999.0 : Math.abs(acceleration);
        this.max_deceleration = (deceleration == 0) ? -9999999999999999.0 : -Math.abs(deceleration);
    }
}
