package org.firstinspires.ftc.teamcode.common.purepursuit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class Pose extends Point {

    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose subtract(Pose other) {
        return new Pose(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public Pose divide(Pose other) {
        return new Pose(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public double radius(){
        return Math.hypot(x, y);
    }

    public double atan(){
        return Math.atan2(x, y);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.2f", x, y, heading);
    }
}