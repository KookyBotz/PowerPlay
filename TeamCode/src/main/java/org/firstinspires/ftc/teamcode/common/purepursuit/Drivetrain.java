package org.firstinspires.ftc.teamcode.common.purepursuit;

public abstract class Drivetrain {
    private double x, y, t;

    void set(double x, double y, double t) {
        this.x = x;
        this.y = y;
        this.t = t;
    }
}
