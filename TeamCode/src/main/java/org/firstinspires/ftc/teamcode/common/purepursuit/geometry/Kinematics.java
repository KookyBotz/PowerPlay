package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Kinematics {

    public static double[] forebar(double x, double y, double l) {

        // TODO: Add checks for edge cases
        double targetInches = Math.sqrt(Math.pow(l, 2) - Math.pow(y, 2));
        double targetRadians = Math.asin(y / l);

        return new double[]{x - targetInches, targetRadians};
    }
}
