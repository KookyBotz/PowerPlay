package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Kinematics {

    private static final double FOREBAR_LENGTH = 9.842;

    public double[] forebar(double x, double y) {
        double targetTheta = Math.sin(y / FOREBAR_LENGTH);
        double targetInches = Math.sqrt(Math.pow(y, 2) - Math.pow(FOREBAR_LENGTH, 2));

        return new double[]{targetInches, targetTheta};
    }
}
