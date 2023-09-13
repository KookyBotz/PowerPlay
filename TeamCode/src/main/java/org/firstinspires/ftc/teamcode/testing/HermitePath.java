package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

import java.util.ArrayList;

public class HermitePath {
    public HermiteInterpolator interpolator;
    private ArrayList<HermitePose> controlPoses = new ArrayList<>();

    public HermitePath() {
        interpolator = new HermiteInterpolator();
    }

    public HermitePath addPose(double x, double y, Vector2D tangent) {
        this.addPose(new HermitePose(x, y, tangent));
        return this;
    }

    public HermitePath addPose(double x, double y, double h, double m) {
        return this.addPose(new HermitePose(x, y, Vector2D.fromHeadingAndMagnitude(h, m)));
    }

    public HermitePath addPose(HermitePose pose) {
        this.controlPoses.add(pose);
        return this;
    }

    public HermitePath construct() {
        if (controlPoses.size() <= 1) throw new IllegalStateException("Need a minimum of two control poses.");
        interpolator = new HermiteInterpolator();
        interpolator.setControlPoses(controlPoses);
        return this;
    }

    public Pose get(double t, int n) {
        if (t <= 0) {
            return startPose();
        } else if (t >= controlPoses.size()) {
            return endPose();
        } else {
            return interpolator.get(t, n);
        }
    }

    public double getHeading(double t) {
        return interpolator.getHeading(t);
    }

    public double curvature(double t) {
        return interpolator.curvature(t);
    }

    public int length() {
        return controlPoses.size() - 1;
    }

    public Pose startPose() {
        return controlPoses.get(0).pose();
    }

    public Pose endPose() {
        return controlPoses.get(length()).pose();
    }
}