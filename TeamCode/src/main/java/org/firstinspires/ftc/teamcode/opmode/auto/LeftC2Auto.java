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

import org.firstinspires.ftc.teamcode.common.commandbase.auto.C2DepositHighCommand;
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

@Autonomous(name = "Left 1+10 C2")
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

            telemetry.addLine("Left C2 Auto 1+10");
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

//        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Pose intermediate = new Pose(0, 51, 0);

        Pose[] pickup = new Pose[]{
                new Pose(1, 54.25, 0),
                new Pose(1, 54.75, 0),
                new Pose(0, 055.75, 0),
                new Pose(0, 56.25, 0),
                new Pose(0, 56.75, 0),

                new Pose(-69, 56.25, Math.PI + 0.015),
                new Pose(-69, 56.75, Math.PI + 0.015),
                new Pose(-69, 57.25, Math.PI + 0.015),
                new Pose(-69, 57.75, Math.PI + 0.015),
                new Pose(-69, 58.25, Math.PI + 0.015),

                //park
                new Pose(-72, 58.25, Math.PI),
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
                new Pose(-24, 47, -Math.PI / 6.5),

                new Pose(-24, 47, -Math.PI / 6.5),
                new Pose(-24, 48, -Math.PI / 6.5),
                new Pose(-24, 49, -Math.PI / 6.5),
                new Pose(-24, 50, -Math.PI / 6.5),

                new Pose(-38, 57, -Math.PI / 2),

                new Pose(-45, 51.2, Math.PI / 6.5 + Math.PI),
                new Pose(-45, 52, Math.PI / 6.5 + Math.PI),
                new Pose(-44.5, 52.1, Math.PI / 6.5 + Math.PI),
                new Pose(-44, 52.6, Math.PI / 6.5 + Math.PI),
                new Pose(-44, 52.7, Math.PI / 6.5 + Math.PI)
        };

        GrabPosition[] grabPositions = new GrabPosition[]{
                new GrabPosition(560, 0, 0.172, 0.37, 0),
                new GrabPosition(560, 0, 0.14, 0.37, 0),
                new GrabPosition(560, 0, 0.11, 0.37, 0),
                new GrabPosition(560, 0, 0.075, 0.37, 20),
                new GrabPosition(560, 0, 0.05, 0.37, 20)
        };

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 2),
                        new InstantCommand(() -> PositionCommand.max_heading = 0.6),
                        new PositionCommand(drivetrain, localizer, intermediate, 0, 1250, voltage()),
                        new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 1),

                        //preload
                        new PositionCommand(drivetrain, localizer, deposit[0], 0, 1250, voltage())
                                .alongWith(new WaitCommand(630).andThen(new C2DepositHighCommand(lift, intake))),

                        //1
                        new PositionCommand(drivetrain, localizer, pickup[0], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[0]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[1], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[1], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[0]).andThen(new C2DepositHighCommand(lift, intake))),
                        //2
                        new PositionCommand(drivetrain, localizer, pickup[1], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[1]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[2], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[2], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[1]).andThen(new C2DepositHighCommand(lift, intake))),
                        //3
                        new PositionCommand(drivetrain, localizer, pickup[2], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[2]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[3], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[3], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[2]).andThen(new C2DepositHighCommand(lift, intake))),
                        //4
                        new PositionCommand(drivetrain, localizer, pickup[3], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[3]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[4], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[4], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[3]).andThen(new C2DepositHighCommand(lift, intake))),
                        //5
                        new PositionCommand(drivetrain, localizer, pickup[4], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[4]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[5], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[5], 0, 2250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[4])
                                        .andThen(new WaitCommand(220).andThen(new C2DepositHighCommand(lift, intake)).andThen(new WaitCommand(20)))),

                        new PositionCommand(drivetrain, localizer, pickup[5], 0, 1800, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[0]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[6], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[6], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[0]).andThen(new C2DepositHighCommand(lift, intake))),

                        new PositionCommand(drivetrain, localizer, pickup[6], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[1]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[7], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[7], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[1]).andThen(new C2DepositHighCommand(lift, intake))),

                        new PositionCommand(drivetrain, localizer, pickup[7], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[2]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[8], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[8], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[2]).andThen(new C2DepositHighCommand(lift, intake))),

                        new PositionCommand(drivetrain, localizer, pickup[8], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[3]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[9], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[9], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[3]).andThen(new C2DepositHighCommand(lift, intake))),

                        new PositionCommand(drivetrain, localizer, pickup[9], 0, 1250, voltage())
                                .alongWith(new WaitCommand(600).andThen(new C2ExtendCommand(intake, grabPositions[4]))),
                        new PositionCommand(drivetrain, localizer, deposit_inter[10], 0, 250, voltage())
                                .andThen(new PositionCommand(drivetrain, localizer, deposit[10], 0, 1250, voltage()))
                                .alongWith(new C2RetractCommand(intake, lift, grabPositions[4]).andThen(new C2DepositHighCommand(lift, intake))),

                        new PositionCommand(drivetrain, localizer, pickup[10], 0, 1250, voltage()),

                        //record
                        new InstantCommand(() -> endtime = timer.milliseconds())
                )
        );

        robot.reset();

        while (opModeIsActive() && !isStopRequested()) {
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

        robot.read(drivetrain, intake, lift);
        robot.loop(new Pose(), drivetrain, intake, lift);
        robot.write(drivetrain, intake, lift);
        robot.clearBulkCache();

    }

    private double voltage() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
}
