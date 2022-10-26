package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CMYSleeveDetection extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Mat newMat = new Mat();
        newMat.convertTo(newMat, CvType.CV_16U);
        for (int i = 0; i < input.height(); i++) {
            for (int j = 0; j < input.width(); j++) {
                double[] channels = input.get(j, i);
                double k = Math.max(channels[0], Math.max(channels[1], channels[2]));
                double cyan = (1 - channels[0] - k) / (1 - k);
                Imgproc.circle(newMat, new Point(j, i), 0, new Scalar(cyan, cyan, cyan));
            }
        }
        return newMat;
    }
}
