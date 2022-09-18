package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    public enum SleeveRotation {
        RED,
        GREEN,
        BLUE
    }

    public static Scalar  lower_yellow_bounds = new Scalar(205, 205, 0, 255),
            upper_yellow_bounds = new Scalar(255, 255, 49, 255);
    public static Scalar  lower_cyan_bounds = new Scalar(205, 205, 0, 255),
            upper_cyan_bounds = new Scalar(255, 255, 49, 255);
    public static Scalar  lower_yellow_bounds = new Scalar(205, 205, 0, 255),
            upper_yellow_bounds = new Scalar(255, 255, 49, 255);

    Mat thresh = new Mat();

    public static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(30, 23);
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);

    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    private volatile SleeveRotation rotation = SleeveRotation.RED;

    @Override
    public Mat processFrame(Mat input) {
//        Core.inRange(input, lower_yellow_bounds, upper_yellow_bounds, thresh);
//        return thresh;
        Core.inRange(input, lower_yellow_bounds, upper_yellow_bounds, input);
        return input;
    }
}
