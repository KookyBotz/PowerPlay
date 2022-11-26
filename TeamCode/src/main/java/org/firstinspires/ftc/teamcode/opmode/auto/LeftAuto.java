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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.ClearFourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;

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
        robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSED);
        robot.lift.update(LiftSubsystem.LatchState.UNLATCHED);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection(new Point(180, 80));
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
            telemetry.addLine("RUNNING LEFT 5 CYCLE");
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            robot.write();
        }

        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();

        waitForStart();
        camera.stopStreaming();

        // TODO: Alter the cycling commands to use the new latch
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(1.6, 57.6, 0), 25000),
                        new PositionCommand(drivetrain, localizer, new Pose(1.6, 57.6, 4.5), 25000),
//                        new InstantCommand(() -> PurePursuitConfig.pCoefficientX = 34),
//                        new InstantCommand(() -> PurePursuitConfig.pCoefficientY = 26),
                        // preload
                        new PositionCommand(drivetrain, localizer, new Pose(-3.19, 58.13, 0), 1750),
                        new PositionCommand(drivetrain, localizer, new Pose(-5, 51, 1.5 * Math.PI), 1250),
//                        new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
//                        new WaitCommand(250),
//                        new ClearFourbarCommand(robot.intake),
//                        new WaitCommand(500),
//                        new LiftCommand(robot, LiftSubsystem.LiftState.HIGH),
////                        new InstantCommand(() -> robot.lift.newProfile(615, 800, 3000)),
//                        new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),
//                        new WaitUntilCommand(() -> robot.lift.getPos() > 570),
//                        new WaitCommand(500),
//                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
////                        new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
//                        new LiftRetractCommand(robot),
//                        new WaitUntilCommand(() -> robot.lift.getPos() < 10),
//
//                        // intake
//                        new InstantCommand(() -> robot.intake.newProfile(405, 800, 3000)),
//                        new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),
//                        new InstantCommand(() -> robot.intake.extendFourbar(4)),
//                        new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
//                        new WaitUntilCommand(() -> robot.intake.getPos() > 350),
//                        new WaitCommand(750),
//                        new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
//                        new WaitCommand(1000),
//
//                        // transfer
//                        new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
//
//                        new WaitCommand(500),
//
//                        new ParallelCommandGroup(
//                                new PositionCommand(drivetrain, localizer, new Pose(-0.5, 60, 4.425), 1000),
//                                new SequentialCommandGroup(
//                                        new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),
//                                        new InstantCommand(() -> robot.intake.newProfile(-5, 800, 3000)),
//                                        new WaitUntilCommand(() -> robot.lift.getPos() < 10),
//                                        new WaitUntilCommand(() -> robot.intake.getPos() < 10),
//                                        new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT)
//                                )
//                        ),

                        // deposit
                        new WaitCommand(7500),
                        new ParallelCommandGroup( // more heading equals going to the right
                                new PositionCommand(drivetrain, localizer, new Pose(-3.19, 58.13, 4.4), 5000) // add , here
//                                new SequentialCommandGroup(
//                                        new WaitCommand(10000),
//                                // 463,
//                                // 451,
//                                // 438,
//                                // 436,
//                                // 434, 0.2
//                                        new AutoCycleCommand(robot, 463, 0.4),
//                                        new AutoCycleCommand(robot, 451, 0.36),
//                                        new AutoCycleCommand(robot, 438, 0.3),
//                                        new AutoCycleCommand(robot, 436, 0.25),
////                                        new InstantCommand(() -> robot.lift.newProfile(615, 800, 3000)),
//                                        new LiftCommand(robot, LiftSubsystem.LiftState.HIGH),
//                                        new WaitUntilCommand(() -> robot.lift.getPos() > 580),
//                                        new WaitCommand(200),
//                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
////                                        new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500))
////                                )
//                        ), // add comma here
//                        new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT),
//
//                        new PositionCommand(drivetrain, localizer,
//                                position == SleeveDetection.ParkingPosition.CENTER ? new Pose(0, 51, 1.5 * Math.PI) :
//                                        position == SleeveDetection.ParkingPosition.RIGHT ? new Pose(26, 48, 1.5 * Math.PI) :
//                                                new Pose(-26, 48, 1.5 * Math.PI), 2000
//                        )
                )
        ));

        while (opModeIsActive()) {
            robot.read();
            CommandScheduler.getInstance().run();
            robot.intake.loop();
            robot.lift.loop();
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
