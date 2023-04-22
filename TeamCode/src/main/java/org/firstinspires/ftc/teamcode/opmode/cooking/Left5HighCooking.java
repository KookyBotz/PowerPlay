package org.firstinspires.ftc.teamcode.opmode.cooking;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.C2DepositHighCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.HighPoleAutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.HighPoleAutoExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.HoldPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.auto.PrecisePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.IntermediateStateCommand;
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

@Autonomous(name = "Auto Left Cooking")
@Config
public class Left5HighCooking extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        PositionCommand.xController.setPIDF(0.02, 0, 0.03, 0);
        PositionCommand.yController.setPIDF(0.02, 0, 0.03, 0);
        PositionCommand.hController.setPIDF(0.3, 0, 0.05, 0);

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

            telemetry.addLine("1+5 LEFT SIDE HIGH");
            telemetry.addData("PARK", robot.sleeveDetection.getPosition());
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, intake, lift);
        }

        SleeveDetection.ParkingPosition position = robot.sleeveDetection.getPosition();

        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new InstantCommand(() -> PositionCommand.ALLOWED_TRANSLATIONAL_ERROR = 1),
                    new InstantCommand(() -> PositionCommand.ALLOWED_HEADING_ERROR = Math.toRadians(2.5)),
                    new PositionCommand(drivetrain, localizer, new Pose(0, 64, 0), 250, robot.getVoltage()),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new LiftCommand(lift, LiftSubsystem.LiftState.HIGH),
                                    new WaitCommand(75),
                                    new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                                    new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                                    new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED),
                                    new WaitUntilCommand(lift::isWithinTolerance),
                                    new WaitCommand(70)
                            ),
                            new WaitCommand(1000)
                    ),
                    new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                    new WaitCommand(20),
                    new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED)),
                    new PrecisePositionCommand(drivetrain, localizer, new Pose[]{new Pose(0.85, 61.75, 0.23)}, 100, robot.getVoltage()),
                    new HighPoleAutoExtendCommand(drivetrain, lift, intake, new GrabPosition(560, 0, 0.163, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                    new PrecisePositionCommand(drivetrain, localizer, new Pose[]{new Pose(0.85, 61.75, 0.23)}, 100, robot.getVoltage()),
                    new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(542, 0, 0.135, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                    new PrecisePositionCommand(drivetrain, localizer, new Pose[]{new Pose(0.85, 61.75, 0.23)}, 100, robot.getVoltage()),
                    new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(533, 0, 0.1, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                    new PrecisePositionCommand(drivetrain, localizer, new Pose[]{new Pose(0.85, 61.75, 0.23)}, 100, robot.getVoltage()),
                    new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(532, 0, 0.07, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                    new PrecisePositionCommand(drivetrain, localizer, new Pose[]{new Pose(0.85, 61.75, 0.23)}, 100, robot.getVoltage()),
                    new HighPoleAutoCycleCommand(drivetrain, lift, intake, new GrabPosition(535, 0, 0.035, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                    new PrecisePositionCommand(drivetrain, localizer, new Pose[]{new Pose(0.85, 61.75, 0.23)}, 100, robot.getVoltage()),
                    new LiftCommand(lift, LiftSubsystem.LiftState.HIGH),
                    new WaitCommand(75),
                    new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                    new WaitUntilCommand(lift::isWithinTolerance),
                    new WaitCommand(100),
                    new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                    new WaitCommand(75),
                    new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED)),
                    new WaitUntilCommand(() -> lift.isWithinTolerance()),
                    new PositionCommand(drivetrain, localizer, new Pose(0, 29.35, Math.PI / 2), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                    new PositionCommand(drivetrain, localizer,
                            (position.equals(SleeveDetection.ParkingPosition.LEFT) ? new Pose(22, 29.35, Math.PI / 2) :
                                    (position.equals(SleeveDetection.ParkingPosition.CENTER) ? new Pose(0, 29.35, Math.PI / 2) :
                                            new Pose(-22, 29.35, Math.PI / 2))), 2000, 2000,
                            hardwareMap.voltageSensor.iterator().next().getVoltage()),
                    new InstantCommand(() -> intake.power = 0),
                    new InstantCommand(() -> lift.power = 0),
                    new InstantCommand(this::requestOpModeStop)
            )
        );

        robot.reset();

        while (opModeIsActive() && !isStopRequested()) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            if (timer.time() > 27 && !Globals.IS_PARKING) {
                CommandScheduler.getInstance().reset();
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new PositionCommand(drivetrain, localizer, new Pose(0, 29.35, Math.PI / 2), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                new PositionCommand(drivetrain, localizer,
                                        (position.equals(SleeveDetection.ParkingPosition.LEFT) ? new Pose(22, 29.35, Math.PI / 2) :
                                                (position.equals(SleeveDetection.ParkingPosition.CENTER) ? new Pose(0, 29.35, Math.PI / 2) :
                                                        new Pose(-22, 29.35, Math.PI / 2))), 2000, 2000,
                                        hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                new IntermediateStateCommand(intake),
                                new LiftCommand(lift, LiftSubsystem.LiftState.RETRACTED)
                        )
                );
            }
            robot.read(drivetrain, intake, lift);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain, intake, lift);
            localizer.periodic();

            robot.write(drivetrain, intake, lift);
            robot.clearBulkCache();
        }
    }
}
