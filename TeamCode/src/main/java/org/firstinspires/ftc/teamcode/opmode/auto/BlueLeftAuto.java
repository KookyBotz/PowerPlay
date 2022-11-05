package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.CycleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPathBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "BlueLeftAuto")
public class BlueLeftAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, true);
        Drivetrain drivetrain = robot.drivetrain;

        Localizer localizer = new TwoWheelLocalizer(
                () -> robot.horizontalEncoder.getPosition(),
                () -> robot.lateralEncoder.getPosition(),
                ()-> robot.getAngle()
        );
        robot.localizer = localizer;
//        SleeveDetection sleeveDetection;
//        OpenCvCamera camera;
//        SleeveDetection.ParkingPosition position = SleeveDetection.ParkingPosition.LEFT;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
////        sleeveDetection = new SleeveDetection();
////        camera.setPipeline(sleeveDetection);
////
////        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
////        {
////            @Override
////            public void onOpened()
////            {
////                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
////            }
////
////            @Override
////            public void onError(int errorCode)
////            {
////
////            }
////        });

        while (!isStarted()) {
            robot.read();
            for (SwerveModule module : robot.drivetrain.modules) {
                module.setTargetRotation(0);
            }
            robot.drivetrain.updateModules();

//            position = sleeveDetection.getPosition();
//            telemetry.addData("Sleeve Position", position);
            telemetry.addLine("RUNNING BLUE 5 CYCLE");
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.write();
        }

        waitForStart();


//        PurePursuitPath preloadPath = new PurePursuitPathBuilder()
//                .setDrivetrain(drivetrain)
//                .setLocalizer(localizer)
//                .setFollowDistance(10)
//                .setStartPosition(new Pose(6, 90, Math.PI))
//                .setMotionProfile(new RisingMotionProfile(0.3, 1))
//                .then(new Pose(6, 90, Math.PI))
//                .then(new Pose(24, 84, Math.PI))
//                .then(new Pose(60, 84, Math.PI))
//                .then(new Pose(60, 108, Math.PI))
//                .then(new Pose(72, 108, Math.PI))
//                .then(new Pose(72, 108, 7 * Math.PI / 6))
//                .build();

//        Pose parkingPose;
//        if (position == SleeveDetection.ParkingPosition.LEFT) {
//            parkingPose = new Pose(60, 132, Math.PI);
//        } else if (position == SleeveDetection.ParkingPosition.CENTER) {
//            parkingPose = new Pose(60, 108, Math.PI);
//        } else {
//            parkingPose = new Pose(60, 84, Math.PI);
//        }
//
//        PurePursuitPath visionPath = new PurePursuitPathBuilder()
//                .setDrivetrain(drivetrain)
//                .setLocalizer(localizer)
//                .setFollowDistance(4)
//                .setMotionProfile(new RisingMotionProfile(0.7, 1))
//                .then(parkingPose)
//                .build();

        PurePursuitPath path = new PurePursuitPath(drivetrain, localizer, true, new RisingMotionProfile(0.3, 0.5),
                new Waypoint(new Pose(6, 0, 0), 5));
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                    new PurePursuitCommand(path)
//                    // cycle
////                    new CycleCommand(robot),
////                    new CycleCommand(robot),
////                    new CycleCommand(robot),
////                    new CycleCommand(robot),
////                    new CycleCommand(robot),
////                    new CycleCommand(robot),
////                    // park
////                    new PurePursuitCommand(visionPath)
//                )
//        );

        while (opModeIsActive()) {
            robot.read();

            path.update();
            CommandScheduler.getInstance().run();
            robot.drivetrain.updateModules();
            localizer.periodic();
            telemetry.addData("current pose", localizer.getPos());
            telemetry.update();

            robot.write();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }
}
