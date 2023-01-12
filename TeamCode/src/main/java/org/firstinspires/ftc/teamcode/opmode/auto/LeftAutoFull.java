package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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

import java.util.function.BooleanSupplier;

@Autonomous(name = "⬅️ LeftAuto 1+10 ⬅️")
@Config
public class LeftAutoFull extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    private double loopTime;
    private BooleanSupplier side_left = () -> true;

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
        robot.intake.update(IntakeSubsystem.PivotState.FLAT);
        robot.intake.update(IntakeSubsystem.TurretState.INTAKE);

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
        robot.startIMUThread(this);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // get to cycle position
                        // start cycling
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PositionCommand(drivetrain, localizer, new Pose(-5, 56.5, 4.49), 500, 3000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new SwerveXCommand(robot.drivetrain)

                                ),

                                new WaitCommand(1700).andThen(new SequentialCommandGroup(
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[0]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[1]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[2]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[3]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[4])
                                ))

                        ),

                        new PositionCommand(drivetrain, localizer, new Pose(-3.5, 52.5, -1.5 * Math.PI), 500, 1250, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(65, 52.5, -1.5 * Math.PI), 500, 1500, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        new InstantCommand(() -> side_left = ()-> false),

                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PositionCommand(drivetrain, localizer, new Pose(65, 61.5, -4.51), 500, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        new SwerveXCommand(robot.drivetrain)
                                ),

                                new WaitCommand(1200).andThen(new SequentialCommandGroup(
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[0]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[1]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[2]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[3]),
                                        new AutoCycleCommand(robot, CYCLE_GRAB_POSITIONS[4]),
                                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                        new LiftPositionCommand(robot.lift, 610, 6000, 7500, 30, 1000, LiftSubsystem.STATE.FAILED_EXTEND)
                                                .alongWith(new PositionCommand(drivetrain, localizer, new Pose(63.5, 57.5, -4.6), 500, 3000, hardwareMap.voltageSensor.iterator().next().getVoltage())),
                                        new WaitCommand(0),
                                        new LiftPositionCommand(robot.lift, -5, 6000, 7500, 10, 1000, LiftSubsystem.STATE.FAILED_RETRACT)
                                                .alongWith(new WaitCommand(50).andThen(new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)))),

                                        new PositionCommand(drivetrain, localizer, new Pose(66, 52, 0), 500, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage())
                                ))

                        ),

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
                                new ConditionalCommand(
                                        // left side cycle position
                                        // -5, 56.5, 4.49
                                        // park offsets
                                        // LEFT -31, 49 (-26
                                        // CENT -5,  49 (0
                                        // RIGH  18, 49 (23
                                        new PositionCommand(drivetrain, localizer,
                                                position == SleeveDetection.ParkingPosition.CENTER ? new Pose(-5, 49, 0) :
                                                        position == SleeveDetection.ParkingPosition.RIGHT ? new Pose(18, 51, 0) :
                                                                new Pose(-31, 49, 0), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()
                                        ), new PositionCommand(drivetrain, localizer,
                                        position == SleeveDetection.ParkingPosition.CENTER ? new Pose(39, 49, 0) :
                                                position == SleeveDetection.ParkingPosition.RIGHT ? new Pose(65, 51, 0) :
                                                        new Pose(88, 49, 0), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                        side_left
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

            telemetry.addData("STATE: ", robot.intake.state);
            telemetry.addData("STATE: ", robot.lift.state);
            telemetry.addData("targetPos", robot.intake.targetPosition);
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
