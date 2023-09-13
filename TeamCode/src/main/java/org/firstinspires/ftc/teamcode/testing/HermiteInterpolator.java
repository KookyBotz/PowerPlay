package org.firstinspires.ftc.teamcode.testing;

import java.util.ArrayList;

import org.ejml.data.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public class HermiteInterpolator {
    private ArrayList<Spline> splines = new ArrayList<>();
    private ArrayList<HermitePose> controlPoses = new ArrayList<>();

    private final double EPSILON = 1e-5;

    private final double[][] hermiteConstants = {
            {1.0, 0.0, 0.0, 0.0},
            {1.0, 1.0, 1.0, 1.0},
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 1.0, 2.0, 3.0}
    };

    public HermiteInterpolator(HermitePose... controlPoses) {
        interpolate();
    }

    private Spline createSpline(HermitePose start, HermitePose end) {
        SimpleMatrix CUBIC_HERMITE_MATRIX = new SimpleMatrix(hermiteConstants);

        double[][] inputs = {
                {start.x, start.y},
                {end.x, end.y},
                {(1 / end.subt(start).add(EPSILON).x) * start.tangent.x, (1 / end.subt(start).add(EPSILON).y) * start.tangent.y},
                {(1 / end.subt(start).add(EPSILON).x) * end.tangent.x, (1 / end.subt(start).add(EPSILON).y) * end.tangent.y}
        };

        SimpleMatrix O = CUBIC_HERMITE_MATRIX.solve(new SimpleMatrix(inputs));

        SimpleMatrix x = O.extractVector(false, 0);
        SimpleMatrix y = O.extractVector(false, 1);

        return new Spline(new Polynomial(x), new Polynomial(y));
    }

    private void interpolate() {
        for (int i = 0; i < controlPoses.size() - 1; i++) {
            splines.add(createSpline(controlPoses.get(i), controlPoses.get(i + 1)));
        }
    }

    public void setControlPoses(ArrayList<HermitePose> controlPoses) {
        this.controlPoses = controlPoses;
        interpolate();
    }

    public Pose get(double t, int n) {
        int splineIndex = (int) Math.floor(t);
        double splineT = t - splineIndex;
        try {
            Point point = splines.get(splineIndex).calculate(splineT, n);
            return new Pose(point.getX(), point.getY(), splines.get(splineIndex).getHeading(splineT));
        } catch (IndexOutOfBoundsException e) {
            if (t >= controlPoses.size() - 1) {
                Point point = splines.get(splines.size() - 1).calculate(1, n);
                return new Pose(point.getX(), point.getY(), splines.get(splines.size() - 1).getHeading(splineT));
            } else {
                Point point = splines.get(0).calculate(1, n);
                return new Pose(point.getX(), point.getY(), splines.get(0).getHeading(splineT));
            }
        }
    }

    public double getHeading(double t) {
        int splineIndex = (int) Math.floor(t);
        double splineT = t - splineIndex;
        return splines.get(splineIndex).getHeading(splineT);
    }

    public double curvature(double t) {
        if (t >= controlPoses.size() - 1) {
            t -= 0.001;
        }
        return splines.get(getSplineIndex(t)).curvature(t - getSplineIndex(t));
    }

    private int getSplineIndex(double t) {
        int splineIndex = (int) Math.floor(t);
        return splineIndex;
    }

    public double headingInterpolator(double initialHeading, double finalHeading, double t) {
        double deltaHeading = finalHeading - initialHeading;

        if (deltaHeading > 180) {
            deltaHeading -= 360;
        } else if (deltaHeading < -180) {
            deltaHeading += 360;
        }

        return (initialHeading + deltaHeading * t) % 360;
    }
}