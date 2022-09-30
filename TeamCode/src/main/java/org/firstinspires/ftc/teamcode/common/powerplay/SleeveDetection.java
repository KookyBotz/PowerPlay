package org.firstinspires.ftc.teamcode.common.powerplay;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
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

    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    public static int COLOR_MAX = 100;
    public static int COLOR_MIN = 200;

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

    private double yelPercent, cyaPercent, magPercent;

    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), blurredMat = new Mat();

    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);

    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    private volatile SleeveRotation rotation = SleeveRotation.YELLOW;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        //blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));
        
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        magPercent = Core.countNonZero(magMat);

        double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));

        if (maxPercent == yelPercent) {
            rotation = SleeveRotation.YELLOW;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == cyaPercent) {
            rotation = SleeveRotation.CYAN;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (maxPercent == magPercent) {
            rotation = SleeveRotation.MAGENTA;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }

        return yelMat;
    }
}
