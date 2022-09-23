package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class VisionConstants {
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    public static int COLOR_MAX = 49;
    public static int COLOR_MIN = 205;

    private static final Scalar
            lower_yellow_bounds  = new Scalar(COLOR_MIN, COLOR_MIN, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, COLOR_MAX, 255),
            lower_cyan_bounds    = new Scalar(0, COLOR_MIN, COLOR_MIN, 255),
            upper_cyan_bounds    = new Scalar(COLOR_MAX, 255, 255, 255),
            lower_magenta_bounds = new Scalar(COLOR_MIN, 0, COLOR_MIN, 255),
            upper_magenta_bounds = new Scalar(255, COLOR_MAX, 255, 255);

    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255),
            WHITE   = new Scalar(255, 255, 255);
}
