package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class PositionCommand extends CommandBase {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.25;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(1);

    public static double hP = 0.4;
    public static double hD = 0.2;
    public static double hI = 0;

    public static double mP = 0.15;
    public static double mD = 0.2;
    public static double mI = 0.0;

    public static PIDFController mController = new PIDFController(mP, mI, mD, 0);
    public static PIDFController hController = new PIDFController(hP, hI, hD, 0);
    public static double max_power = 0.7;
    public static double max_heading = 0.5;

    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;
    ElapsedTime deadTimer;

    private final double ms;
    private final double delay;
    private ElapsedTime delayTimer;

    private final double v;
    private final boolean lock;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double dead, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = dead;
        this.delay = delay;
        this.v = voltage;
        this.lock = false;
    }

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = Integer.MAX_VALUE;
        this.delay = delay;
        this.v = voltage;
        this.lock = true;
    }

    @Override
    public void execute() {
        if (deadTimer == null) {
            deadTimer = new ElapsedTime();
        }

        Pose powers = goToPosition(localizer.getPos(), targetPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        Pose error = targetPose.subtract(localizer.getPos());
        Globals.error = error;
        Globals.targetPose = targetPose;

        boolean reached = ((Math.hypot(error.x, error.y) < ALLOWED_TRANSLATIONAL_ERROR) && (Math.abs(error.heading) < ALLOWED_HEADING_ERROR));
        Globals.reached = reached;

        if (reached && delayTimer == null) {
            delayTimer = new ElapsedTime();
        }
        if (!reached && delayTimer != null) {
            delayTimer.reset();
        }

        boolean delayed = delayTimer != null && delayTimer.milliseconds() > delay;
        return (deadTimer.milliseconds() > ms) || delayed;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = targetPose.subtract(robotPose);

        double x_rotated = deltaPose.x * cos(robotPose.heading) - deltaPose.y * sin(robotPose.heading);
        double y_rotated = deltaPose.x * sin(robotPose.heading) + deltaPose.y * cos(robotPose.heading);

        double magnitude = hypot(y_rotated, x_rotated);
        double dir = atan2(y_rotated, x_rotated);

        double power = mController.calculate(0, magnitude);

        if (Math.abs(power) < 0.075) power = 0;

        double y_component = cos(dir) * power;
        double x_component = sin(dir) * power;
        double heading_component = hController.calculate(0, deltaPose.heading);

        if (Math.abs(heading_component) < 0.015) heading_component = 0;

        Pose powers = new Pose(
                x_component / v * 12.5,
                -y_component / v * 12.5,
                -heading_component / v * 12.5
        );

        return new Pose(powers.x, powers.y, powers.heading);
    }
}
