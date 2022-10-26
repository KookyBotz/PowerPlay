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
//        Mat newMat = new Mat();
//        newMat.convertTo(input, CvType.CV_16U);
//        for (int i = 0; i < input.height(); i++) {
//            for (int j = 0; j < input.width(); j++) {
////                double[] colors = input.get(j, i);
////                double k = Math.max(colors[0], Math.max(colors[1], colors[2]));
////                double cyan = (1 - colors[0] - k) / (1 - k);
//                input.put(j, i);
//            }
//        }
        for (int i = 0; i < input.height(); i++) {
            for (int j = 0; j < input.width(); j++) {
                double[] colors = input.get(j, i);
                try {
                    double k = Math.max(colors[0], Math.max(colors[1], colors[2]));
                    int cyan = (int)((1 - colors[0] - k) / (1 - k));
                    input.put(i, j, 255, 255, 255, 0);
                } catch (Exception e) {
                    input.put(i, j, 255, 255, 255, 255);
                }


            }
        }
        return input;
    }
}
