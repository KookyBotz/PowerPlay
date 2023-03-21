package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
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

    // Lift Subsystem Constants
    public static double LIFT_HIGH_POS = 37.34;
    public static double LIFT_MID_POS = 25.43;
    public static double LIFT_RETRACT_POS = 0;

    public static double LIFT_MAX_V = 0.0;
    public static double LIFT_MAX_A = 0.0;
    public static double LIFT_MAX_D = 0.0;

    public static double LIFT_LATCHED = 0;
    public static double LIFT_UNLATCHED = 0;

    public static final double LIFT_TICKS_PER_INCH = 22.754;

    public static double LIFT_MANUAL_FACTOR = 1;
    public static double LIFT_MAX = 21.97;

    public static double LIFT_EXTENDED_TOLERANCE = 1.32;

    // Intake Subsystem Constants
    public static double INTAKE_EXTENSION_MAX_V = 0;
    public static double INTAKE_EXTENSION_MAX_A = 0;
    public static double INTAKE_EXTENSION_MAX_D = 0;

    public static double INTAKE_FOURBAR_MAX_V = 6;
    public static double INTAKE_FOURBAR_MAX_A = 9999;
    public static double INTAKE_FOURBAR_MAX_D_UP = 3;
    public static double INTAKE_FOURBAR_MAX_D_DOWN = 3;


    public static double INTAKE_CLAW_OPEN = 0.48;
    public static double INTAKE_CLAW_CLOSED = 0.57;

    public static double INTAKE_PIVOT_LOW = 0.335;
    public static double INTAKE_PIVOT_FLAT = 0.48;
    public static double INTAKE_PIVOT_PRE_TRANSFER = 0.592;
    public static double INTAKE_PIVOT_TRANSFER = 0.592;
    public static double INTAKE_PIVOT_PICKUP = 0.48;

    public static double INTAKE_TURRET_OUTWARDS = 0.728;
    public static double INTAKE_TURRET_INTERMEDIATE = 0.449;
    public static double INTAKE_TURRET_INWARDS = 0.17;

    public static double INTAKE_FOURBAR_PRE_TRANSFER = 0.391;
    public static double INTAKE_FOURBAR_TRANSFER = 0.469;
    public static double INTAKE_FOURBAR_INTERMEDIATE = 0.385;
    public static double INTAKE_FOURBAR_INTAKE = 0.06;
    public static double INTAKE_FOURBAR_LOW = 0.359;
    public static double INTAKE_FOURBAR_GROUND = 0.075;

    public static final long INTAKE_CLAW_CLOSE_TIME = 84; // ms

    public static final double EXTENSION_TICKS_PER_INCH = 23.54;

    public static double INTAKE_MANUAL_FACTOR = 10;
    public static double INTAKE_TURRET_FACTOR = 0.007;
    public static double INTAKE_FOURBAR_FACTOR = -0.00025;
    public static double INTAKE_PIVOT_FACTOR = 0.03;
    public static double INTAKE_MIN = 0;
    public static double INTAKE_MAX = 23.73;

    public static double INTAKE_EXTENDED_TOLERANCE = 1.32;

    public static Side SIDE = Side.LEFT;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;

    public static int wait1 = 1000;
    public static int wait2 = 1000;
    public static int wait3 = 1000;
    public static int wait4 = 200;
    public static int wait5 = 50;
}
