package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class TapeDetection extends OpenCvPipeline {
    public TapeDetection() {

    }

    private static final Scalar
            lower_red_bounds = new Scalar(200, 0, 0),
            upper_red_bounds = new Scalar(255, 100, 100),
            lower_blue_bounds = new Scalar(0, 0, 200),
            upper_blue_bounds = new Scalar(100, 100, 255);

    private final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255);

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }
}
