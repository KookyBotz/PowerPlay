package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public enum Side {
        LEFT,
        RIGHT
    }

    public static int LEFTSIDE_REGION_X = 75;
    public static int LEFTSIDE_REGION_Y = 120;

    public static int RIGHTSIDE_REGION_X = 75;
    public static int RIGHTSIDE_REGION_Y = 120;

    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    public static Side side = Side.LEFT;
}
