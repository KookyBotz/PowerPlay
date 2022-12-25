package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.SwerveXCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.FileInterface;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem.CYCLE_GRAB_POSITIONS;

@Autonomous(name = "⬅️ LeftAuto ⬅️")
@Config
public class LeftAuto extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    private double loopTime;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, true);
        Drivetrain drivetrain = robot.drivetrain;

        Localizer localizer = new TwoWheelLocalizer(
                () -> robot.horizontalEncoder.getPosition(),
                () -> robot.lateralEncoder.getPosition(),
                robot::getAngle
        );

        robot.localizer = localizer;
        robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.lift.update(LiftSubsystem.LatchState.LATCHED);
        robot.intake.update(IntakeSubsystem.ClawState.OPEN);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        int cameraMonitorViewId =   hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection(new Point(90, 80));
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

            telemetry.addLine("RUNNING LEFT 5 CYCLE");
            telemetry.addData("PARK", sleeveDetection.getPosition());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.write();
        }

        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();

        waitForStart();
        camera.stopStreaming();
        robot.startIMUThread(this);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // get to cycle position
                        new PositionCommand(drivetrain, localizer, new Pose(-5, 57.98, 0), 2500),
                        new PositionCommand(drivetrain, localizer, new Pose(-5, 57.98, 4.48), 4000),

                        // start cycling
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[0], true),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[1], true),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[2], true),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[3], true),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[4], false),

                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                        new InstantCommand(() -> robot.intake.setFourbar(robot.intake.fourbar_transition)),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE)),

                                        new WaitCommand(250),

                                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                        new LiftPositionCommand(robot.lift, 610, 3000, 7500, 30, 3000, LiftSubsystem.STATE.FAILED_EXTEND),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                        new LiftPositionCommand(robot.lift, 0, 3000, 7500, 10, 2000, LiftSubsystem.STATE.FAILED_RETRACT)

                                ),
                                new SequentialCommandGroup(
                                        new SwerveXCommand(robot.drivetrain),
                                        new PositionCommand(drivetrain, localizer, new Pose(-2.5, 55, 4), 1500)
                                )

                        ),
                        new PositionCommand(drivetrain, localizer,
                                position == SleeveDetection.ParkingPosition.CENTER ? new Pose(-5, 49, 0) :
                                        position == SleeveDetection.ParkingPosition.RIGHT ? new Pose(18, 51, 0) :
                                                new Pose(-31, 49, 0), 2000
                        ),
                        new InstantCommand(() -> robot.intake.setFourbar(robot.intake.fourbar_retracted)),
                        new WaitCommand(500),
                        new InstantCommand(this::requestOpModeStop)
                )
        );

        robot.reset();

        while (opModeIsActive()) {
            robot.read();

            if (robot.intake.state == IntakeSubsystem.STATE.FAILED_RETRACT || robot.lift.state == LiftSubsystem.STATE.FAILED_RETRACT) {
                CommandScheduler.getInstance().reset();
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new PositionCommand(drivetrain, localizer,
                                        position == SleeveDetection.ParkingPosition.CENTER ? new Pose(-5, 49, 0) :
                                                position == SleeveDetection.ParkingPosition.RIGHT ? new Pose(21, 51, 0) :
                                                        new Pose(-31, 49, 0), 2000
                                ),
                                new InstantCommand(this::requestOpModeStop)
                        )
                );
            }

            CommandScheduler.getInstance().run();
            robot.intake.loop();
            robot.lift.loop();
            robot.drivetrain.updateModules();
            localizer.periodic();

//            telemetry.addData("STATE: ", robot.intake.state);
//            telemetry.addData("STATE: ", robot.lift.state);
            telemetry.addData("targetPos", robot.intake.extensionTargetPosition);
            telemetry.addData("intakePos", robot.intake.getPos());
            telemetry.addData("current pose", localizer.getPos());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write();
            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
