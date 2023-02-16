package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class ProfileState {
    public double x;
    public double v;
    public double a;

    public ProfileState(double position, double velocity, double acceleration) {
        this.x = position;
        this.v = velocity;
        this.a = acceleration;
    }
}
