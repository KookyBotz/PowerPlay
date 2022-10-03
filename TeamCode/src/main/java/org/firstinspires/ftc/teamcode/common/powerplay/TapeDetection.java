package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class TapeDetection extends OpenCvPipeline {
    public TapeDetection() {

    }

    private static final Scalar
            lower_red_bounds = new Scalar(200, 0, 0),
            upper_red_bounds = new Scalar(255, 100, 100);

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }
}
