package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;


@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private ElapsedTime timer;
    private double loopTime = 0;

    private boolean xLock = false;
    private boolean rumble = true;

    private boolean lastBButton2 = false;
    private boolean lastAButton2 = false;
    private boolean lastXButton2 = false;
    private boolean lastYButton2 = false;

    private boolean lastRightBumper2 = false;
    private boolean lastRightTrigger1 = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

         AUTO = false;

        robot.init(hardwareMap);
        robot.intake = new IntakeSubsystem();
//        robot.lift = new LiftSubsystem();
        robot.drivetrain = new SwerveDrivetrain();
//        robot.localizer = new TwoWheelLocalizer();
        robot.enabled = true;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();

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
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier * ((robot.intake.isExtended) ? 0.5 : 1)
        );

        /*
         * Depositing - G1
         */
        boolean rightTrigger1 = gamepad1.right_trigger > 0.15;
        if (rightTrigger1 && !lastRightTrigger1) {
            if (robot.intake.fourbarState.equals(IntakeSubsystem.FourbarState.LOW) ||
                robot.intake.fourbarState.equals(IntakeSubsystem.FourbarState.GROUND)) {
                // deposit cone, and go to intermediate position with intake turret
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.update(ClawState.OPEN)),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.OUTWARDS))
                        )
                );
            } else if (robot.lift.getPos() > LIFT_EXTENDED_TOLERANCE && robot.lift.latchState.equals(LiftSubsystem.LatchState.LATCHED)) {
                // TODO retract slides, open latch
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED))
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
            if (robot.intake.hasCone()) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> robot.intake.update(ClawState.CLOSED)),
                                                new WaitCommand(INTAKE_CLAW_CLOSE_TIME)
                                        ), null,
                                        () -> robot.intake.clawState.equals(IntakeSubsystem.ClawState.OPEN)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.DEPOSIT)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INWARDS))
                        )
                );
            } else {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.INTAKE)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new InstantCommand(() -> robot.intake.update(ClawState.OPEN))
                        )
                );
            }
        }
        lastRightBumper2 = rightBumper;

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
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.LOW)),
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.LOW))
                    )
            );
        } else if (BButton2 && !lastBButton2) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                            new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.GROUND))
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

        robot.loop(drive);
        CommandScheduler.getInstance().run();
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return Math.signum(num) * min + (1 - min) * Math.pow(Math.abs(num), 3) * Math.signum(num);
    }
}
