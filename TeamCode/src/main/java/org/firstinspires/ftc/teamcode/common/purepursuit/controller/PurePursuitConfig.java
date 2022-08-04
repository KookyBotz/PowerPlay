package org.firstinspires.ftc.teamcode.common.purepursuit.controller;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitConfig {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(5);

    public static int pCoefficientX = 26;
    public static int pCoefficientY = 24;
    public static double pCoefficientH = Math.PI / 2.5;
}
