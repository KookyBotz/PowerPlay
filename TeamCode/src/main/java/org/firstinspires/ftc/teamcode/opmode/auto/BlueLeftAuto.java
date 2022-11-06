package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.ClearFourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueLeftAuto")
public class BlueLeftAuto extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, true);
        Drivetrain drivetrain = robot.drivetrain;
        ElapsedTime timer = new ElapsedTime();

        Localizer localizer = new TwoWheelLocalizer(
                () -> robot.horizontalEncoder.getPosition(),
                () -> robot.lateralEncoder.getPosition(),
                robot::getAngle
        );
        robot.localizer = localizer;
        robot.intake.closeForebar();
        robot.intake.closeClaw();
//        SleeveDetection sleeveDetection;
//        OpenCvCamera camera;
//        SleeveDetection.ParkingPosition position = SleeveDetection.ParkingPosition.LEFT;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection(new Point(195, 80));
//        sleeveDetection.setBoundingBox(new Point(195, 80));
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            robot.read();
            for (SwerveModule module : robot.drivetrain.modules) {
                module.setTargetRotation(0);
            }
            robot.drivetrain.updateModules();

            telemetry.addData("Sleeve Position", sleeveDetection.getPosition());
            telemetry.addLine("RUNNING BLUE 5 CYCLE");
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.write();
        }

        waitForStart();

        PurePursuitPath preloadPath = new PurePursuitPath(drivetrain, localizer, true, new RisingMotionProfile(0.5, 0.5),
                new Waypoint(new Pose(0, 0, 0), 0),
                new Waypoint(new Pose(0, 63, 0), 0),
                new Waypoint(new Pose(0, 63, 1.5 * Math.PI), 0)
        );

        PurePursuitPath intakePath = new PurePursuitPath(drivetrain, localizer, true, new RisingMotionProfile(0.5, 0.5),
                new Waypoint(new Pose(0, 64, 1.5 * Math.PI), 0),
                new Waypoint(new Pose(-5, 51, 1.5 * Math.PI), 0)
        );

        PurePursuitPath depositPath = new PurePursuitPath(drivetrain, localizer, true, new RisingMotionProfile(0.5, 0.5),
                new Waypoint(new Pose(-5, 51, 1.5 * Math.PI), 0),
                new Waypoint(new Pose(0, 60, Math.PI + 1.35), 0)
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // preload
                        new PurePursuitCommand(preloadPath),
                        new InstantCommand(() -> PurePursuitConfig.pCoefficientX = 24),
                        new InstantCommand(() -> PurePursuitConfig.pCoefficientY = 24),
                        new InstantCommand(() -> robot.intake.openClaw()),
                        new WaitCommand(500),
                        new ClearFourbarCommand(robot.intake),
                        new WaitUntilCommand(() -> robot.intake.getFourbarPos() <= robot.intake.fourbar_transition),
                        new WaitCommand(750),
                        new InstantCommand(() -> robot.lift.newProfile(610, 800, 3000)),
                        new InstantCommand(() -> robot.intake.intakeTurret()),
                        new WaitUntilCommand(() -> robot.lift.getPos() > 570),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
                        new WaitUntilCommand(() -> robot.lift.getPos() < 10),

                        // intake
                        new PurePursuitCommand(intakePath),
                        new InstantCommand(() -> robot.intake.newProfile(405, 800, 3000)),
                        new InstantCommand(() -> robot.intake.intakeTurret()),
                        new InstantCommand(() -> robot.intake.extendForebar(4)),
                        new InstantCommand(() -> robot.intake.openClaw()),
                        new WaitUntilCommand(() -> robot.intake.getPos() > 390),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.intake.closeClaw()),
                        new WaitCommand(1000),

                        // transfer
                        new InstantCommand(() -> robot.intake.transitionFourbar()),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.intake.depositTurret()),
                        new InstantCommand(() -> robot.intake.newProfile(-5, 800, 3000)),
                        new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                        new WaitUntilCommand(() -> robot.intake.getPos() < 10),
                        new InstantCommand(() -> robot.intake.closeForebar()),

                        // deposit
                        new PurePursuitCommand(depositPath),

                        new AutoCycleCommand(robot, 480, 0.22),
                        new AutoCycleCommand(robot, 467, 0.19),
                        new AutoCycleCommand(robot, 474, 0.14),
                        new AutoCycleCommand(robot, 474, 0.1),
                        new InstantCommand(() -> robot.lift.newProfile(610, 800, 3000)),
                        new WaitUntilCommand(() -> robot.lift.getPos() > 580),
                        new WaitCommand(250),
                        new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500))
                    )
        );


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

        while (opModeIsActive()) {
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.read();
            CommandScheduler.getInstance().run();
            robot.intake.loop();
            robot.lift.loop();
            robot.drivetrain.updateModules();
            localizer.periodic();
            telemetry.addData("current pose", localizer.getPos());
            telemetry.update();
            robot.write();
        }
    }
}
