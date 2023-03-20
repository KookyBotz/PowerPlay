package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    private boolean xLock = false;
    private boolean rumble = true;
    private boolean lastBButton = false;
    private boolean lastAButton = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        AUTO = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.intake = new IntakeSubsystem();
            robot.lift = new LiftSubsystem();
            robot.drivetrain = new SwerveDrivetrain();
            robot.localizer = new TwoWheelLocalizer();

//            robot.startIMUThread(this);
        }

//        robot.read();

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
         * Field centric with speed control
         */
        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3)),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3))).rotate(0),
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier * ((robot.intake.isExtended) ? 0.5 : 1)
        );

        /*
         * Depositing
         */
        if (gamepad1.right_trigger > 0.15) {
            if (robot.intake.fourbarState.equals(IntakeSubsystem.FourbarState.LOW) ||
                robot.intake.fourbarState.equals(IntakeSubsystem.FourbarState.GROUND)) {
                // deposit cone, and go to intermediate position with intake turret
            } else if (robot.lift.getPos() > LIFT_EXTENDED_TOLERANCE) {
                // retract slides, open latch
            }
        }

        boolean aButton = gamepad2.a;
        if (aButton && !lastAButton) {
            robot.intake.newProfile(INTAKE_FOURBAR_INTERMEDIATE, 0, 0, 0, IntakeSubsystem.ProfileTarget.INTAKE);
        }
        lastAButton = aButton;

        boolean bButton = gamepad2.b;
        if (bButton && !lastBButton) {
            robot.intake.newProfile(INTAKE_FOURBAR_INTAKE, 0, 0, 0, IntakeSubsystem.ProfileTarget.INTAKE);
        }
        lastBButton = bButton;

        robot.loop(drive);
        CommandScheduler.getInstance().run();
//        robot.write();

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
