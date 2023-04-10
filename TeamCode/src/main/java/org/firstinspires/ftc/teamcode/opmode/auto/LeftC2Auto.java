package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.C2DepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.C2ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.C2RetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;

@Autonomous(name = "Left C2 Auto")
@Config
public class LeftC2Auto extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private SleeveDetection sleeveDetection;
    private double loopTime;
    private double endtime = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.SIDE = Globals.Side.LEFT;
        Globals.AUTO = true;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        intake = new IntakeSubsystem(robot);
        lift = new LiftSubsystem(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        while (!isStarted()) {
            robot.read(drivetrain, intake, lift);
            for (SwerveModule module : drivetrain.modules) {
                module.setTargetRotation(Math.PI / 2);
            }
            drivetrain.updateModules();

            telemetry.addLine("Left C2 Auto");
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

//        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Pose intermediate = new Pose(0, 55, 0);

        Pose[] pickup = new Pose[]{
                new Pose(3.5, 54, -0.055),
                new Pose(3, 54.5, -0.055),
                new Pose(2, 55.5, -0.055),
                new Pose(2, 56, -0.055),
                new Pose(2, 57, -0.055),
                new Pose(-70.5, 56, Math.PI),
                new Pose(-70.5, 56.5, Math.PI),
                new Pose(-70.5, 57, Math.PI),
                new Pose(-70.5, 57.5, Math.PI),
                new Pose(-70.5, 58, Math.PI),
        };

        Pose[] deposit_inter = new Pose[]{
                new Pose(-27.66, 51, 0),
                new Pose(-27, 51.6, 0),
                new Pose(-27.66, 52.2, 0),
                new Pose(-27.66, 52.8, 0),
                new Pose(-27.66, 53.4, 0),
                new Pose(-27.33, 54, 0),
                new Pose(-43.5, 53, Math.PI),
                new Pose(-43.5, 53.6, Math.PI),
                new Pose(-43.5, 54.2, Math.PI),
                new Pose(-43.5, 54.8, Math.PI),
                new Pose(-43.5, 55.4, Math.PI)
        };

        Pose[] deposit = new Pose[]{
                new Pose(-27, 46, -Math.PI / 6),
                new Pose(-27, 46.1, -Math.PI / 6),
                new Pose(-27.66, 46.7, -Math.PI / 6),
                new Pose(-27.66, 48.9, -Math.PI / 6),
                new Pose(-27.66, 49.5, -Math.PI / 6),
                new Pose(-36.5, 56, -Math.PI / 2),
                new Pose(-43.5, 52, Math.PI / 6 + Math.PI),
                new Pose(-43.5, 52.6, Math.PI / 6 + Math.PI),
                new Pose(-43.5, 53.2, Math.PI / 6 + Math.PI),
                new Pose(-43.5, 53.8, Math.PI / 6 + Math.PI),
                new Pose(-43.5, 54.4, Math.PI / 6 + Math.PI)
        };

        GrabPosition[] grabPositions = new GrabPosition[]{
                new GrabPosition(560, 0, 0.172, 0.37, 0),
                new GrabPosition(560, 0, 0.139, 0.37, 0),
                new GrabPosition(560, 0, 0.106, 0.37, 0),
                new GrabPosition(560, 0, 0.075, 0.37, 20),
                new GrabPosition(560, 0, 0.035, 0.37, 20)
        };

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 2),
                        new InstantCommand(() -> PositionCommand.max_heading = 0.6),
                        new PositionCommand(drivetrain, localizer, intermediate, 0, 1150, voltage()),
                        new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 1),

                        //preload
                        new PositionCommand(drivetrain, localizer, deposit[0], 0, 1250, voltage())
                                .alongWith(new WaitCommand(725).andThen(new C2DepositCommand(lift))),

                        //1
                        new PositionCommand(drivetrain, localizer, pickup[0], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[0]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[1], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[1], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[0]).andThen(new C2DepositCommand(lift))),
                        //2
                        new PositionCommand(drivetrain, localizer, pickup[1], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[1]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[2], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[2], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[1]).andThen(new C2DepositCommand(lift))),
                        //3
                        new PositionCommand(drivetrain, localizer, pickup[2], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[2]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[3], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[3], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[2]).andThen(new C2DepositCommand(lift))),
                        //4
                        new PositionCommand(drivetrain, localizer, pickup[3], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[3]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[4], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[4], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[3]).andThen(new C2DepositCommand(lift))),
                        //5
                        new PositionCommand(drivetrain, localizer, pickup[4], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[4]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[5], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[5], 0, 2250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[4]).andThen(new WaitCommand(100).andThen(new C2DepositCommand(lift)))),

                        new PositionCommand(drivetrain, localizer, pickup[5], 0, 1750, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[0]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[6], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[6], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[0]).andThen(new C2DepositCommand(lift))),

                        new PositionCommand(drivetrain, localizer, pickup[6], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[1]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[7], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[7], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[1]).andThen(new C2DepositCommand(lift))),

                        new PositionCommand(drivetrain, localizer, pickup[7], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[2]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[8], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[8], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[2]).andThen(new C2DepositCommand(lift))),

                        new PositionCommand(drivetrain, localizer, pickup[8], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[3]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[9], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[9], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[3]).andThen(new C2DepositCommand(lift))),

                        new PositionCommand(drivetrain, localizer, pickup[9], 0, 1250, voltage())
                                .alongWith(new WaitCommand(650).andThen(new C2ExtendCommand(intake, grabPositions[4]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[10], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[10], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, grabPositions[4]).andThen(new C2DepositCommand(lift))),

                        //record
                        new InstantCommand(() -> endtime = timer.milliseconds())
                )
        );

        robot.reset();

        while (opModeIsActive()) {
            if (timer == null) {
                timer = new ElapsedTime();
            }
            robot.read(drivetrain, intake, lift);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain, intake, lift);
            localizer.periodic();

            telemetry.addData("encoder pod", intake.getPos());
            telemetry.addData("time", endtime);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }

    private double voltage() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
}
