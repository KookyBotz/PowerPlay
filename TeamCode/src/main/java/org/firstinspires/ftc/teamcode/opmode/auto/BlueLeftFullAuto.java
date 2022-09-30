package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPathBuilder;

@Autonomous (name = "BlueLeftFullAuto")
public class BlueLeftFullAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Localizer localizer = new BetterSwerveLocalizer(() -> -robot.getAngle() + Math.PI, robot.drivetrain.modules);

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

        PurePursuitPath path = new PurePursuitPathBuilder()
                .setDrivetrain(drivetrain)
                .setLocalizer(localizer)
                .setController(true)
                .setFollowDistance(10)
                .setStartPosition(new Pose(6, 90, Math.PI))
                .setPower(0.7)
                .setMotionProfile(new RisingMotionProfile(0.7, 1))
                .then(new Pose(6, 90, Math.PI))
                .then(new Pose(24, 84, Math.PI))
                .then(new Pose(60, 84, Math.PI))
                .then(new Pose(60, 115, 7 * Math.PI / 6))
                .build();
//                .setFollowDistance(0)
//                .then(new Pose(60, 123, 7 * Math.PI / 6))
//                .then(new Pose(60, 123, Math.PI))
//                .then(new Pose(60, 123, 7 * Math.PI / 6))
//                .then(new Pose(60, 123, Math.PI))
//                .then(new Pose(60, 123, 7 * Math.PI / 6))
//                .then(new Pose(60, 123, Math.PI))
//                .then(new Pose(60, 123, 7 * Math.PI / 6))
//                .then(new Pose(60, 123, Math.PI))
//                .then(new Pose(60, 123, 7 * Math.PI / 6))
//                .then(new Pose(60, 123, Math.PI))
//                .then(new Pose(60, 123, 7 * Math.PI / 6))
                // cycle other side
        PurePursuitPath path2 = new PurePursuitPathBuilder()
                .setDrivetrain(drivetrain)
                .setLocalizer(localizer)
                .setController(true)
                .setFollowDistance(10)
                .setPower(0.7)
                .setMotionProfile(new RisingMotionProfile(0.7, 1))
                .then(new Pose(60, 115, 7 * Math.PI / 6))
                .then(new Pose(60, 12, 0))
//                .then(new Pose(60, 24, -Math.PI / 6))
//                .then(new Pose(60, 12, 0))
//                .then(new Pose(60, 24, -Math.PI / 6))
//                .then(new Pose(60, 12, 0))
//                .then(new Pose(60, 24, -Math.PI / 6))
//                .then(new Pose(60, 12, 0))
//                .then(new Pose(60, 24, -Math.PI / 6))
//                .then(new Pose(60, 12, 0))
//                .then(new Pose(60, 24, -Math.PI / 6))
//                .setFollowDistance(10)
                .then(new Pose(60, 123, 0))

                .build();

        // go to preload position path
        // once there, extend up,
        // as soon as cone released, extend intake
        // grab cone
        // retract to bring cone back
        // extend up
        // repeat
        // park in designated area

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PurePursuitCommand(path),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 115, 7 * Math.PI / 6)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, Math.PI)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, 7 * Math.PI / 6)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, Math.PI)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, 7 * Math.PI / 6)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, Math.PI)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, 7 * Math.PI / 6)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, Math.PI)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, 7 * Math.PI / 6)),
                        new WaitCommand(1000),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 110, Math.PI)),
                        new WaitCommand(1000),
                        new PurePursuitCommand(path2)
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

        CommandScheduler.getInstance().reset();
    }
}
