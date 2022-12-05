package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.FileWriteCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.SwerveXCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.FileInterface;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "LeftAuto")
@Config
public class LeftAuto extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;

    boolean start = true;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.write();
        }

        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();

        waitForStart();
        camera.stopStreaming();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(-4.5, 57.98, 0), 2500),
                        new PositionCommand(drivetrain, localizer, new Pose(-4.5, 57.98, 4.49), 3000),
                        // sin of heading times 23.7
                        // x error times sin of heading times 23.4
                        // error will be negative for left side auto
                        // subtract from slide position
                        // x error / sin of heading times 23.4
                        // subtract slide pos from error in x
                        // generate a pose for where the cones are
                        // set extension to magnitude of that length - a certain offset * times per inches
                        // or not
                        new WaitCommand(100),

                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AutoCycleCommand(robot, robot.intake.kinematicStates[0], true),
                                        new AutoCycleCommand(robot, robot.intake.kinematicStates[1], false),
                                        new AutoCycleCommand(robot, robot.intake.kinematicStates[2], false),
                                        new AutoCycleCommand(robot, robot.intake.kinematicStates[3], false),
                                        new AutoCycleCommand(robot, robot.intake.kinematicStates[4], false),

                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                                                        new InstantCommand(() -> robot.intake.setFourbar(robot.intake.fourbar_transition)),
                                                        new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),

                                                        new WaitCommand(250),
                                                        new LatchCommand(robot, LiftSubsystem.LatchState.LATCHED),

                                                        new LiftCommand(robot, LiftSubsystem.LiftState.HIGH),

                                                        //wait until ready to intake
                                                        new WaitUntilCommand(() -> robot.lift.getPos() > 580),
                                                        new WaitCommand(750),

                                                        new LatchCommand(robot, LiftSubsystem.LatchState.UNLATCHED),
                                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),

                                                        new WaitUntilCommand(() -> robot.lift.getPos() < 100)
                                                ),
                                                new PositionCommand(drivetrain, localizer, new Pose(-1, 57.98, 4.49), 1000)
                                        )
                                ),
                                new SwerveXCommand(robot.drivetrain)
                        ),

                        new PositionCommand(drivetrain, localizer,
                                position == SleeveDetection.ParkingPosition.CENTER ? new Pose(-5, 51, 1.5 * Math.PI) :
                                        position == SleeveDetection.ParkingPosition.RIGHT ? new Pose(21, 48, 1.5 * Math.PI) :
                                                new Pose(-31, 48, 1.5 * Math.PI), 2000
                        ),
                        new InstantCommand(() -> robot.intake.setFourbar(IntakeSubsystem.fourbar_retracted)),
                        new WaitCommand(500),
                        new FileWriteCommand(FileInterface.INTAKE, robot.intake.getPos()),
                        new FileWriteCommand(FileInterface.LIFT, robot.lift.getPos()),
                        new FileWriteCommand(FileInterface.IMU, robot.getAngle()),
                        new InstantCommand(this::requestOpModeStop)
                )
        );

        while (opModeIsActive()) {
            if (start) {
                start = false;
                robot.reset();
            }
            robot.read();
            CommandScheduler.getInstance().run();
            robot.intake.loop();
            robot.lift.loop();
            robot.drivetrain.updateModules();
            localizer.periodic();
            telemetry.addData("targetPos", robot.intake.targetPosition);
            telemetry.addData("intakePos", robot.intake.getPos());
            telemetry.addData("current pose", localizer.getPos());
            telemetry.update();
            robot.write();
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }
}
