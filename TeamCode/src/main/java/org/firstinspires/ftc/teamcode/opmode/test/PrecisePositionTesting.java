package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem.CYCLE_GRAB_POSITIONS;

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PrecisePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.SlowAutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.function.BooleanSupplier;

@TeleOp(name = "PRECISE AUTO TESTING")
@Config
public class PrecisePositionTesting extends LinearOpMode {

    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    private double loopTime;
    private BooleanSupplier side_left = () -> true;
//    private boolean schedule = true;

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
//        sleeveDetection = new SleeveDetection(new Point(75, 120));
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
                module.setTargetRotation(Math.PI/2);
            }
            robot.drivetrain.updateModules();

            telemetry.addLine("RUNNING LEFT 5 CYCLE HIGH");
            telemetry.addLine(sleeveDetection.getPosition().toString());
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
                new PrecisePositionCommand(drivetrain, localizer, new Pose(0, 0, 0), 500, 75000, hardwareMap.voltageSensor.iterator().next().getVoltage())
        );

        robot.reset();

        while (opModeIsActive()) {
            robot.read();

            CommandScheduler.getInstance().run();
            robot.intake.loop();
            robot.lift.loop();
            robot.drivetrain.updateModules();
            localizer.periodic();

            telemetry.addData("targetPos", robot.intake.targetPosition);
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
