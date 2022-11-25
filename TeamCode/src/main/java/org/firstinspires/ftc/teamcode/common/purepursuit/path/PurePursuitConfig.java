package org.firstinspires.ftc.teamcode.common.purepursuit.path;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
public class PurePursuitConfig {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.5;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(3);

    public static double pCoefficientX = 0.0;
    public static double pCoefficientY = 0.0;
    public static double pCoefficientH = 0.0;

    // 24, 26 for mecanum, -Math.PI / 2.5
    // look into possibly seeing if p values are too aggressive
    public static double xP = 0.1;
    public static double xD = 0.5;
    // 0.13 * 0.75
    public static double xF = 0.03;

    public static double yP = 0.1;
    public static double yD = 0.5;

    // 0.014 * 0.75
    public static double yF = 0.03;

    public static double hP = -0.6;
    public static double hD = -1.2;
    public static double hF = 0;

//    public static double hP = 0.0;
//    public static double hD = 0;
//    public static double hF = 0;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);
    public static double max_power = 0.7;
}
