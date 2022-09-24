package org.firstinspires.ftc.teamcode.common.powerplay;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RelocalizerDetection extends OpenCvPipeline {

    public static int COLOR_MAX = 49;
    public static int COLOR_MIN = 205;

    private static final Scalar
            lower_red_bounds  = new Scalar(COLOR_MIN, 0, 0, 255),
            upper_red_bounds  = new Scalar(255, COLOR_MAX, COLOR_MAX, 255);
//            lower_blue_bounds = new Scalar(0, 0, COLOR_MIN, 255),
//            upper_blue_bounds = new Scalar(COLOR_MAX, COLOR_MAX, 255, 255);

    public RelocalizerDetection() {}

    @Override
    public Mat processFrame(Mat input) {

        return input;
    }
}
