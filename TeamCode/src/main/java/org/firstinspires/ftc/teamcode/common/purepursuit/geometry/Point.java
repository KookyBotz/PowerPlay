package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

public class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point subtract(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public double distanceTo(Point other) {
        return other.subtract(this).radius();
    }

    public double atan(){
        return Math.atan2(x, y);
    }

    public double radius() {
        return Math.hypot(x, y);
    }
}
