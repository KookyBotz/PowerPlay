package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public class HermitePose {
    public double x, y;
    public Vector2D tangent;

    public HermitePose(double x, double y, Vector2D tangent) {
        this.x = x;
        this.y = y;
        this.tangent = tangent;
    }

    public Point add(HermitePose other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point subt(HermitePose other) {
        return new Point(x - other.x, y - other.y);
    }

    public Pose pose() {
        return new Pose(x, y, Math.atan2(y, x));
    }

    @Override
    public String toString() {
        return String.format("%.2f, %.2f, %s", x, y, tangent);
    }
}
