package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Kinematics {

    public static double[] forebar(double x, double y, double l) {

        // TODO: Add checks for edge cases
        double targetInches = Math.sqrt(Math.pow(l, 2) - Math.pow(y, 2));
        double targetRadians = Math.asin(y / l);

        if (y < 0){
            targetRadians = map(targetRadians, 0, Math.PI / 2, 0.1, 0.61);
        }
        // check quadrant
        // based on that then map it between different number of values

        return new double[]{x - targetInches, targetRadians};
    }

    public static double map(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
