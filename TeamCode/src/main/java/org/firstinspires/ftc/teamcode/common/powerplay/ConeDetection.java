package org.firstinspires.ftc.teamcode.common.powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ConeDetection extends OpenCvPipeline {

    public static boolean active = false;
    public static Alliance alliance = Alliance.RED;

    private final int WINDOW_WIDTH = 320;
    private final int WINDOW_HEIGHT = 240;

    private final Scalar WHITE = new Scalar(255, 255, 255);

    private Mat locMat = new Mat();
    private Rect coneRect = new Rect();

    public enum Alliance {
        RED,
        BLUE
    }

    @Override
    public Mat processFrame(Mat input) {
        // red or blue image processing stuff to isolate cone
        Core.extractChannel(input, locMat, (alliance == Alliance.RED) ? 0 : 2);
        // TODO: Tune this
        Imgproc.threshold(locMat, locMat, 100, 240, 0);

        // noise reduction (ignore cones in the background)
        // TODO: Tune this
        Imgproc.GaussianBlur(locMat, locMat, new Size(5, 5), 0);
        // (If needed apply morphology to this)

        // find cone
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(locMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) {
            coneRect = new Rect();
            return input;
        }

        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();

            // TODO: Tune this
            if (contourArray.length > 20) {
                MatOfPoint2f points = new MatOfPoint2f(contourArray);
                coneRect = Imgproc.boundingRect(points);
                points.release();
            }
            contour.release();
        }
        locMat.release();

        // return image with contours drawn
        Imgproc.drawContours(input, contours, -1, WHITE);
        Imgproc.rectangle(input, coneRect, WHITE);

        return input;
    }

    // if cone is on the left, rotate left
    // if cone is on the right, rotate right
    public double getCone() {
        double xPoint = coneRect.x + (coneRect.width) / 2;
        // double newXPoint = map(0, 320, -180, 180, xPoint);
        return xPoint;
//        return coneRect.x + (coneRect.width / 2);
    }
}