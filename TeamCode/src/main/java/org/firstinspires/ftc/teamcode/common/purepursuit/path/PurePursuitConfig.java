package org.firstinspires.ftc.teamcode.common.purepursuit.path;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitConfig {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(5);

    // 24, 26 for mecanum, -Math.PI / 2.5
    // look into possibly seeing if p values are too aggressive
    public static int pCoefficientX = 18;
    public static int pCoefficientY = 18;
    public static double pCoefficientH = -Math.PI;

    // inches per sec
    public static double MAX_SPEED_VERTICAL = 72;
    public static double MAX_SPEED_HORIZONTAL = 72;
    public static double MAX_SPEED_ROBOT = 72;
}
