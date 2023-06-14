package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

import java.util.Locale;

@Config
public class HoldPositionCommand extends CommandBase {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double PUSHED_THRESHOLD_TRANSLATIONAL = 2;
    public static double PUSHED_THRESHOLD_ROTATIONAL = 0.1;
    public static double ALLOWED_HEADING_ERROR = 0.05;

    public static double xP = 0.05;
    public static double xD = 0.06;
    public static double xI = 0;

    public static double yP = 0.05;
    public static double yD = 0.06;
    public static double yI = 0;

    public static double hP = 0.4;
    public static double hD = 0.2;
    public static double hI = 0;

    public static double mP = 0.15;
    public static double mD = 0.0;
    public static double mI = 0.0;


    public static PIDFController xController = new PIDFController(xP, xI, xD, 0);
    public static PIDFController yController = new PIDFController(yP, yI, yD, 0);
    public static PIDFController hController = new PIDFController(hP, hI, hD, 0);
    public static PIDFController mController = new PIDFController(mP, mI, mD, 0);


    public static double max_power = 1;

    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;
    ElapsedTime deadTimer;

    private final double ms;
    private ElapsedTime delayTimer;

    private final double v;

    public static boolean retargeting = false;

    public HoldPositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double ms, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = ms;
        this.v = voltage;

        Globals.AUTO = true;
    }

    @Override
    public void execute() {
        if (deadTimer == null) {
            deadTimer = new ElapsedTime();
        }
        Pose error = targetPose.subtract(localizer.getPos());
        retargeting =
                Math.abs(error.x) > PUSHED_THRESHOLD_TRANSLATIONAL
                        || Math.abs(error.y) > PUSHED_THRESHOLD_TRANSLATIONAL
                        || Math.abs(error.heading) > PUSHED_THRESHOLD_ROTATIONAL;


        boolean reached = Math.abs(error.x) < ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(error.y) < ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(error.heading) < ALLOWED_HEADING_ERROR;

        if (reached)
            retargeting = false;


        if (Math.abs(xController.getPositionError()) < ALLOWED_TRANSLATIONAL_ERROR)
            xController.reset();

        if (Math.abs(yController.getPositionError()) < ALLOWED_TRANSLATIONAL_ERROR)
            yController.reset();

        if (Math.abs(hController.getPositionError()) < ALLOWED_HEADING_ERROR)
            hController.reset();


        if (retargeting) {
            Pose powers = goToPosition(localizer.getPos(), targetPose);
            drivetrain.set(powers);
            ((SwerveDrivetrain) drivetrain).updateModules();
            Globals.yummypose = powers;
        }
        System.out.println(retargeting);

        if (reached) {
            SwerveDrivetrain drivetrain = (SwerveDrivetrain) this.drivetrain;
            drivetrain.set(new Pose());
            drivetrain.frontLeftModule.setTargetRotation(PI / 10);
            drivetrain.frontRightModule.setTargetRotation(-PI / 10);
            drivetrain.backRightModule.setTargetRotation(PI / 10);
            drivetrain.backLeftModule.setTargetRotation(-PI / 10);
        }

    }

    @Override
    public boolean isFinished() {
        return delayTimer != null && delayTimer.milliseconds() > ms;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
        Globals.AUTO = false;
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = targetPose.subtract(robotPose);

        double x_rotated = deltaPose.x * cos(robotPose.heading) - deltaPose.y * sin(robotPose.heading);
        double y_rotated = deltaPose.x * sin(robotPose.heading) + deltaPose.y * cos(robotPose.heading);

        double magnitude = hypot(y_rotated, x_rotated);
        double dir = atan2(y_rotated, x_rotated);

        double power = mController.calculate(0, magnitude);

        double x_component = cos(dir) * power;
        double y_component = sin(dir) * power;

        System.out.println(String.format(Locale.ENGLISH, "mag: %.2f, dir: %.2f, pow: %.2f, x: %.2f, y: %.2f", magnitude, dir, power, x_component, y_component));

        Pose powers = new Pose(
                x_component,
                y_component,
                hController.calculate(0, deltaPose.heading)
        );

        return new Pose(powers.x / v * 12.5, powers.y / v * 12.5, -powers.heading / v * 12.5);
    }

    public static Pose pos_lock(Pose robotPose, Pose targetPose, double voltage) {
        Pose deltaPose = targetPose.subtract(robotPose);

        double x_rotated = deltaPose.x * cos(robotPose.heading) - deltaPose.y * sin(robotPose.heading);
        double y_rotated = deltaPose.x * sin(robotPose.heading) + deltaPose.y * cos(robotPose.heading);

        double magnitude = hypot(y_rotated, x_rotated);
        double dir = atan2(y_rotated, x_rotated);

        double power = mController.calculate(0, magnitude);

        double y_component = cos(dir) * power;
        double x_component = sin(dir) * power;

        System.out.println(String.format(Locale.ENGLISH, "mag: %.2f, dir: %.2f, pow: %.2f, x: %.2f, y: %.2f", magnitude, dir, power, x_component, y_component));

        Pose powers = new Pose(
                x_component / voltage * 12.5,
                -y_component / voltage * 12.5,
                -hController.calculate(0, deltaPose.heading) / voltage * 12.5
        );

        return new Pose(powers.x, powers.y, powers.heading);
    }
}
