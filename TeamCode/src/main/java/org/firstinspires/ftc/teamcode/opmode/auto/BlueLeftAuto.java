package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.CycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.ScoreCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPathBuilder;

@Autonomous (name = "BlueLeftAuto")
public class BlueLeftAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Localizer localizer = new BetterSwerveLocalizer(() -> -robot.getAngle(), robot.drivetrain.modules);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        while (!isStarted()) {
            for (SwerveModule module : robot.drivetrain.modules) {
                module.setTargetRotation(0);
            }
            robot.drivetrain.updateModules();
            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

        waitForStart();
        robot.startIMUThread(this);

        PurePursuitPath preloadPath = new PurePursuitPathBuilder()
                .setDrivetrain(drivetrain)
                .setLocalizer(localizer)
                .setFollowDistance(10)
                .setStartPosition(new Pose(6, 90, Math.PI))
                .setMotionProfile(new RisingMotionProfile(0.7, 1))
                .then(new Pose(6, 90, Math.PI))
                .then(new Pose(24, 84, Math.PI))
                .then(new Pose(60, 84, Math.PI))
                .then(new Pose(60, 108, Math.PI))
                .then(new Pose(72, 108, Math.PI))
                .then(new Pose(72, 108, 7 * Math.PI / 6))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new PurePursuitCommand(preloadPath),
                    // cycle
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new ScoreCommand(robot)
                )
        );

        while (opModeIsActive()) {
            localizer.periodic();
            CommandScheduler.getInstance().run();
            robot.drivetrain.updateModules();

            telemetry.addData("current pose", localizer.getPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
