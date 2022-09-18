package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum SleeveRotation {
        YELLOW,
        CYAN,
        MAGENTA
    }

    public static Scalar lower_yellow_bounds  = new Scalar(205, 205, 0, 255),
                         upper_yellow_bounds  = new Scalar(255, 255, 49, 255),
                         lower_cyan_bounds    = new Scalar(0, 205, 205, 255),
                         upper_cyan_bounds    = new Scalar(49, 255, 255, 255),
                         lower_magenta_bounds = new Scalar(205, 0, 205, 255),
                         upper_magenta_bounds = new Scalar(255, 49, 255, 255);

    public static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(30, 23);

    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    public static final int COLOR_MAX = 49;
    public static final int COLOR_MIN = 205;

    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat();

    Telemetry telemetry = new Telemetry();

    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);

    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    private volatile SleeveRotation rotation = SleeveRotation.YELLOW;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, input, new Size(5, 5));
        Core.inRange(input, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        telemetry.addData("val:", Core.countNonZero(yelMat));
        telemetry.update();
        return yelMat;
    }
}
