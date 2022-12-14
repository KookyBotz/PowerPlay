package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point(){
        this(0, 0);
    }

    public Point subtract(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point add(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point divide(double div){
        return new Point(x / div, y / div);
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

    public static Point polar(double r, double a){
        return new Point(Math.cos(a)*r, Math.sin(a)*r);
    }

    public Point rotate(double amount){
        return Point.polar(radius(), atan()+amount);
    }
    
    
}
