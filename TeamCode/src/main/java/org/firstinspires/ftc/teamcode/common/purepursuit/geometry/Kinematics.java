package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Kinematics {

    private static final double FOREBAR_LENGTH = 9.842;

    public double[] forebar(double x, double y) {
        double targetInches = Math.sqrt(Math.pow(FOREBAR_LENGTH, 2) - Math.pow(y, 2));
        double targetRadians = Math.asin(y / FOREBAR_LENGTH);

        return new double[]{x - targetInches, targetRadians};
    }
}
