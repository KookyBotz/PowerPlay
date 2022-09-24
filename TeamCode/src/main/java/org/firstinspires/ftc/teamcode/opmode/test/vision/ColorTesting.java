package org.firstinspires.ftc.teamcode.opmode.test.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorTesting extends LinearOpMode {
    int width = 320, height = 240;
    OpenCvCamera cam;
    RelocalizerDetection pipeline;
    
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RelocalizerDetection();
        cam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        cam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            // ftc dashboard stuff

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class RelocalizerDetection extends OpenCvPipeline {

        public static int COLOR_MAX = 49;
        public static int COLOR_MIN = 205;

//    private static final Scalar
//            lower_red_bounds  = new Scalar(COLOR_MIN, 0, 0, 255),
//            upper_red_bounds  = new Scalar(255, COLOR_MAX, COLOR_MAX, 255);
//            lower_blue_bounds = new Scalar(0, 0, COLOR_MIN, 255),
//            upper_blue_bounds = new Scalar(COLOR_MAX, COLOR_MAX, 255, 255);

        public static Scalar lower_red = new Scalar(COLOR_MIN, 0, 0, 255);
        public static Scalar upper_red = new Scalar(255, COLOR_MAX, COLOR_MAX, 255);

        public RelocalizerDetection() {}

        @Override
        public void init(Mat frame) {}

        @Override
        public Mat processFrame(Mat input) {

            return input;
        }
    }
}
