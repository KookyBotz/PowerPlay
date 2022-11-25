package org.firstinspires.ftc.teamcode.opmode.test.Auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PositionCommand;
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
public class TuningAuto extends LinearOpMode {


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
        robot.lift.update(LiftSubsystem.LatchState.UNLATCHED);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

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
        Pose targetPose = new Pose(0, 10, 0);

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, targetPose, 1750)
                ) // test
        );

        while (opModeIsActive()) {
            robot.read();
            CommandScheduler.getInstance().run();
            robot.intake.loop();
            robot.lift.loop();
            robot.drivetrain.updateModules();
            localizer.periodic();
            telemetry.addData("current pose", localizer.getPos());
            telemetry.addData("current y", localizer.getPos().y);
            telemetry.addData("error", localizer.getPos().subtract(targetPose).y);
            telemetry.addData("final", targetPose.y);
            telemetry.update();
            robot.write();
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }
}
