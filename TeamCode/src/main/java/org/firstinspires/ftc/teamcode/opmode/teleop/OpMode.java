package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem.ClawState;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private ElapsedTime timer;
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
    private boolean lastRightTrigger1 = false;
    private boolean lastLeftBumper2 = false;

    private boolean lastRightTrigger2 = false;
    private boolean lastLeftTrigger2 = false;
    
    private boolean clawHasCone = false;

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.AUTO = false;

        robot.init(hardwareMap, telemetry);
        intake = new IntakeSubsystem(robot);


        robot.enabled = true;

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            } catch (Exception e) {}
        }

//        robot.read();
        clawHasCone = !robot.clawSensor.getState();

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

        /*
         * Robot centric with speed control - G1
         */
        double speedMultiplier = 1 - 0.75 * gamepad1.left_trigger;
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3)),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3))).rotate(0),
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier * ((intake.isExtended) ? 0.5 : 1)
        );

        /*
         * Depositing - G1
         */
        boolean rightTrigger1 = gamepad1.right_trigger > 0.15;
        if (rightTrigger1 && !lastRightTrigger1) {
            if (intake.fourbarState.equals(IntakeSubsystem.FourbarState.LOW) ||
                intake.fourbarState.equals(IntakeSubsystem.FourbarState.GROUND)) {
                // deposit cone, and go to intermediate position with intake turret
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(ClawState.OPEN)),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS))
                        )
                );
            } else if (lift.getPos() > Globals.LIFT_EXTENDED_TOLERANCE && lift.latchState.equals(LiftSubsystem.LatchState.LATCHED)) {
                // TODO retract slides, open latch
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED))
                        )
                );
            }
        }
        lastRightTrigger1 = rightTrigger1;

        /*
         * Transfer to Intake Sequences - G2
         */
        boolean rightBumper = gamepad2.right_bumper;
        if (!lastRightBumper2 && rightBumper) {
            if (clawHasCone) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.PRE_TRANSFER)),
                                new WaitCommand(Globals.wait5),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.PRE_TRANSFER)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INWARDS)),
                                new WaitCommand(Globals.wait1),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.TRANSFER)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.TRANSFER)),
                                new WaitCommand(Globals.wait2),
                                new InstantCommand(() -> intake.update(ClawState.OPEN)),
                                new WaitCommand(Globals.wait3),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new WaitCommand(Globals.wait4),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS))
                        )
                );
            } else {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTAKE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> intake.update(ClawState.OPEN))
                        )
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
            if (clawHasCone) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INTERMEDIATE)),
                                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT))
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
                    new SequentialCommandGroup(
                            new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                            new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INTERMEDIATE)),
                            new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT))
                    )
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
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.LOW)),
                            new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                            new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.LOW))
                    )
            );
        } else if (BButton2 && !lastBButton2) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                            new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                            new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.GROUND))
                    )
            );
        } else if (YButton2 && !lastYButton2) {
            // high pole
        } else if (XButton2 && !lastXButton2) {
            // mid pole
        }
        lastAButton2 = AButton2;
        lastBButton2 = BButton2;
        lastXButton2 = XButton2;
        lastYButton2 = YButton2;

        /*
         * MANUAL CONTROLS
         */
        double leftX = gamepad2.left_stick_x;
        if (Math.abs(leftX) > 0.4) {
            intake.setFourbarFactor(joystickScalar(leftX, 0.4));
        }

        double rightX = gamepad2.right_stick_x;
        if (Math.abs(rightX) > 0.15) {
            intake.setTurretFactor(joystickScalar(rightX, 0.15));
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

        intake.loop2();

        CommandScheduler.getInstance().run();

        try {
            robot.loop(drive, drivetrain, intake, lift);
            robot.write(drivetrain, intake, lift);

        } catch (Exception e) {

        }

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("fourbarPos", robot.fourbarLeft.getPosition());
        telemetry.addData("pivotPos", robot.pivot.getPosition());
        telemetry.addData("turretPos", robot.turret.getPosition());
        telemetry.addData("clawPos", robot.claw.getPosition());
        telemetry.addData("sensor", clawHasCone);
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return Math.signum(num) * min + (1 - min) * Math.pow(Math.abs(num), 3) * Math.signum(num);
    }
}
