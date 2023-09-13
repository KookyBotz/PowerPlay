package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;

public class Spline {
    private Polynomial x, y;

    public Spline(Polynomial x, Polynomial y) {
        this.x = x;
        this.y = y;
    }

    public Point calculate(double t, int n) {
        return new Point(x.calculate(t, n), y.calculate(t, n));
    }

    public double curvature(double t) {
        double x_prime = x.calculate(t, 1);
        double y_prime = y.calculate(t, 1);

        double kNum = Math.abs((y.calculate(t, 2) * x_prime) - y_prime * x.calculate(t, 2));
        double kDom = Math.pow((x_prime * x_prime) + (y_prime * y_prime), 1.5);

        return kNum / kDom;
    }

    public double getHeading(double t) {
        return Math.atan2(y.calculate(t, 1), x.calculate(t, 1));
    }
}
