package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.SwerveXCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.GroundScoreCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.IntermediateStateCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.LowScoreCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem.ClawState;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private ElapsedTime timer;
    private ElapsedTime timer2;
    private double loopTime = 0;

    private boolean xLock = false;
    private boolean rumble = true;

    private boolean lastBButton2 = false;
    private boolean lastAButton2 = false;
    private boolean lastXButton2 = false;
    private boolean lastYButton2 = false;
    private boolean lastYButton1 = false;
    private boolean lastAButton1 = false;

    private boolean lastRightBumper2 = false;
    private boolean lastRightBumper1 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastLeftBumper1 = false;

    private boolean lastDpadLeft1 = false;
    private boolean lastDpadRight2 = false;

    private boolean lastRightTrigger2 = false;
    private boolean lastLeftTrigger2 = false;

    private boolean lastGamepadUp1 = false;
    private boolean lastGamepadDown1 = false;
    
    private boolean clawHasCone = false;
    public static double position;

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USING_IMU = false;

        robot.init(hardwareMap, telemetry);
        intake = new IntakeSubsystem(robot);
        lift = new LiftSubsystem(robot);
        drivetrain = new SwerveDrivetrain(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            try{
                robot.reset();
//                robot.startIMUThread(this);
            } catch (Exception e) {}
//            SwerveDrivetrain.imuOffset = -Math.PI / 2;
        }

        robot.read(drivetrain, intake, lift);

        /*
         * RUMBLE TIMERS
         * Short rumble, then long rumble
         */
        if (timer.seconds() < 45 && timer.seconds() > 30 && rumble) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            rumble = false;
        } else if (timer.seconds() < 30 & !rumble) {
            gamepad1.rumble(2000);
            gamepad1.rumble(2000);
            rumble = true;
        }

        boolean gamepadUp1 = gamepad1.dpad_up;
        boolean gamepadDown1 = gamepad1.dpad_down;
        if (gamepadUp1 && !lastGamepadUp1) {
            lift.update(LiftSubsystem.LatchState.INTERMEDIATE);
        } else if (gamepadDown1 && !lastGamepadDown1) {
            lift.update(LiftSubsystem.LatchState.UNLATCHED);
        }
        boolean lastGamepadUp1 = gamepadUp1;
        boolean lastGamepadDown1 = gamepadDown1;

        /*
         * Robot Centric to Field Centric
         */
//        boolean leftBumper1 = gamepad1.left_bumper;
//        if (leftBumper1 && !lastLeftBumper1) {
//            Globals.USING_IMU = !Globals.USING_IMU;
//        }
//        lastLeftBumper1 = leftBumper1;

//        boolean dpadLeft1 = gamepad1.dpad_left;
//        if (dpadLeft1 && !lastDpadLeft1 && Globals.USING_IMU) {
//            SwerveDrivetrain.imuOffset = robot.getAngle() + Math.PI;
//        }
//        lastDpadLeft1 = dpadLeft1;

//        double extended = intake.getPos() > 200 || lift.getPos() > 75 ? 0.5 : 1;
        double extended = (gamepad1.dpad_left ? 1 : 0.5);
        SwerveDrivetrain.maintainHeading = (Math.abs(gamepad1.left_stick_x) < 0.02 & Math.abs(gamepad1.left_stick_y) < 0.02 & Math.abs(gamepad1.right_stick_x) < 0.02);
        double rotationAmount = (Globals.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.01 ? gamepad1.left_stick_y : 0, 3)),
                        (-Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.01 ? gamepad1.left_stick_x : 0), 3))).rotate(rotationAmount),
                -(Math.pow(-gamepad1.right_stick_x, 3)) * extended
        );

        /*
         * Depositing - G1
         */
        boolean rightBumper1 = gamepad1.right_bumper;
        if (rightBumper1) {
            if (lift.getPos() > Globals.LIFT_EXTENDED_TOLERANCE && lift.latchState == LiftSubsystem.LatchState.INTERMEDIATE && lift.isWithinTolerance()) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                new WaitCommand(75),
                                new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
                        )
                );
            } else if ((intake.fourbarState.equals(IntakeSubsystem.FourbarState.LOW) ||
                    intake.fourbarState.equals(IntakeSubsystem.FourbarState.GROUND))) {
                // deposit cone, and go to intermediate position with intake turret
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(ClawState.OPEN)),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setTargetPosition(-10)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS))
                        )
                );
            }
        }

        if (gamepad1.x) {
            robot.intakeEncoder.reset();
        }

        /*
         * Transfer to Intake Sequences - G2
         */
        boolean rightBumper = gamepad2.right_bumper;
        if (!lastRightBumper2 && rightBumper) {
            if (intake.hasCone()) {
                CommandScheduler.getInstance().schedule(new TransferCommand(intake, lift));
            } else if (!intake.fourbarState.equals(IntakeSubsystem.FourbarState.INTAKE)){
                CommandScheduler.getInstance().schedule(
                        new IntakeStateCommand(intake)
                );
            }
        }
        lastRightBumper2 = rightBumper;

        /*
         * INTAKING AND AUTO TRANSFER
         */
        boolean rightTrigger2 = gamepad2.right_trigger > 0.3;
        boolean leftTrigger2 = gamepad2.left_trigger > 0.3;
        if (rightTrigger2 && !lastRightTrigger2) {
            CommandScheduler.getInstance().schedule(
                    new InstantCommand(() -> intake.update(ClawState.CLOSED))
            );
            if (intake.hasCone()) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME),
                                new WaitUntilCommand(() -> intake.getTargetPosition() <= Globals.INTAKE_EXTENDED_TOLERANCE)
                                        .alongWith(new InstantCommand(() -> intake.setTargetPosition(0)))
                                        .alongWith(new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)))
                                        .alongWith(new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INTERMEDIATE)))
                                        .alongWith(new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)))
                        )
                );
            }
        } else if (leftTrigger2 && !lastLeftTrigger2) {
            CommandScheduler.getInstance().schedule(
                    new InstantCommand(() -> intake.update(ClawState.OPEN))
            );
        }
        lastLeftTrigger2 = leftTrigger2;
        lastRightTrigger2 = rightTrigger2;

        boolean leftBumper2 = gamepad2.left_bumper;
        if (leftBumper2 && !lastLeftBumper2) {
            CommandScheduler.getInstance().schedule(
                    new IntermediateStateCommand(intake)
            );
        }
        lastLeftBumper2 = leftBumper2;

        /*
         * GOING TO SCORING POSITION - G2
         */
        boolean AButton2 = gamepad2.a;
        boolean BButton2 = gamepad2.b;
        boolean XButton2 = gamepad2.x;
        boolean YButton2 = gamepad2.y;
        if (AButton2 && !lastAButton2) {
            CommandScheduler.getInstance().schedule(new LowScoreCommand(intake));
        } else if (BButton2 && !lastBButton2) {
            CommandScheduler.getInstance().schedule(new GroundScoreCommand(intake));
        } else if (XButton2 && !lastXButton2) {
            CommandScheduler.getInstance().schedule(new LiftCommand(lift, LiftSubsystem.LiftState.MID)
                    .alongWith(new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED))
                    .alongWith(new WaitUntilCommand(() -> Math.abs(lift.getPos() - lift.getTargetPos()) < 20))
                    .andThen(new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE)));
        } else if (YButton2 && !lastYButton2) {
            CommandScheduler.getInstance().schedule(new LiftCommand(lift, LiftSubsystem.LiftState.HIGH)
                    .alongWith(new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED))
                    .alongWith(new WaitUntilCommand(() -> Math.abs(lift.getPos() - lift.getTargetPos()) < 20))
                    .andThen(new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE)));
        }
        lastAButton2 = AButton2;
        lastBButton2 = BButton2;
        lastXButton2 = XButton2;
        lastYButton2 = YButton2;

        /*
         * MANUAL CONTROLS
         */
        double rightY = gamepad2.right_stick_y;
        if (Math.abs(rightY) > 0.4) {
            intake.setFourbarFactor(joystickScalar(rightY, 0.4));
        }

        double rightX = gamepad2.right_stick_x;
        if (Math.abs(rightX) > 0.15) {
            intake.setTurretFactor(joystickScalar(rightX, 0.15));
        }

        double leftY = gamepad2.left_stick_y;
        if (Math.abs(leftY) > 0.1 && Globals.MANUAL_ENABLED) {
            intake.setSlideFactor(joystickScalar(-leftY, 0.1));
        }

        boolean buttonA1 = gamepad1.a;
        boolean buttonY1 = gamepad1.y;
        if (buttonA1 && !lastAButton1) {
            intake.adjustPivotOffset(-Globals.INTAKE_PIVOT_FACTOR);
            intake.update(intake.pivotState);
        } else if (buttonY1 && !lastYButton1) {
            intake.adjustPivotOffset(Globals.INTAKE_PIVOT_FACTOR);
            intake.update(intake.pivotState);
        }
        lastYButton1 = buttonA1;
        lastAButton1 = buttonY1;

        CommandScheduler.getInstance().run();

        robot.loop(drive, drivetrain, intake, lift);
        robot.write(drivetrain, intake, lift);

        if (gamepad2.dpad_left) {
            robot.latch.setPosition(position);
        }

        boolean dpadRight2 = gamepad2.dpad_right;
        if (dpadRight2 && !lastDpadRight2) {
            schedule(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new AutoCycleCommand(lift, intake, new GrabPosition(560, 0, 0.172, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                    new AutoCycleCommand(lift, intake, new GrabPosition(542, 0, 0.139, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                    new AutoCycleCommand(lift, intake, new GrabPosition(533, 0, 0.106, 0.37, 0), LiftSubsystem.LiftState.HIGH),
                                    new AutoCycleCommand(lift, intake, new GrabPosition(532, 0, 0.075, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                    new AutoCycleCommand(lift, intake, new GrabPosition(535, 0, 0.035, 0.37, 20), LiftSubsystem.LiftState.HIGH),
                                    new SequentialCommandGroup(
                                            new LiftCommand(lift, LiftSubsystem.LiftState.HIGH)
                                                    .alongWith(new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED)),
                                            new WaitUntilCommand(lift::isWithinTolerance),
                                            new LatchCommand(lift, LiftSubsystem.LatchState.UNLATCHED),
                                            new WaitCommand(75),
                                            new LiftCommand(lift, LiftSubsystem.LiftState.RETRACTED)
                                    )
                            ),
                            new SwerveXCommand(drivetrain)
                    )

            );
        }
        lastDpadRight2 = dpadRight2;

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("fl", drivetrain.frontLeftModule.wheelFlipped + " " + drivetrain.frontLeftModule.lastMotorPower);
//        telemetry.addData("fr", drivetrain.frontRightModule.wheelFlipped + " " + drivetrain.frontRightModule.lastMotorPower);
//        telemetry.addData("bl", drivetrain.backLeftModule.wheelFlipped + " " + drivetrain.backLeftModule.lastMotorPower);
//        telemetry.addData("br", drivetrain.backRightModule.wheelFlipped + " " + drivetrain.backRightModule.lastMotorPower);
//        telemetry.addData("maintainHeading", SwerveDrivetrain.maintainHeading);
//        telemetry.addData("liftPow", lift.getPower());
//        telemetry.addData("liftPos", lift.getPos());
//        telemetry.addData("liftTarget", lift.getTargetPos());
//telemetry.addData("fourbarPos", robot.fourbarLeft.getPosition());
//////        telemetry.addData("intakePow", intake.getPower());
//        telemetry.addData("intakePos", intake.getPos());
//        telemetry.addData("intakeTarget", intake.getTargetPosition());
//        telemetry.addData("pivot", robot.pivot.getPosition());
//        telemetry.addData("velocity", robot.extension.getVelocity());
//        telemetry.addData("velocityencoder", robot.intakeEncoder.getRawVelocity());
//        telemetry.addData("intakeCurrent", robot.extension.motorEx.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("hasCone", intake.hasCone());
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return Math.signum(num) * min + (1 - min) * Math.pow(Math.abs(num), 3) * Math.signum(num);
    }
}
