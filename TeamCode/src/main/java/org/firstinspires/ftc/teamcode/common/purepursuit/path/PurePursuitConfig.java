package org.firstinspires.ftc.teamcode.common.purepursuit.path;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitConfig {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(5);

    // 24, 26 for mecanum, -Math.PI / 2.5
    public static int pCoefficientX = 15;
    public static int pCoefficientY = 15;
    public static double pCoefficientH = -Math.PI / 1.5;
}
