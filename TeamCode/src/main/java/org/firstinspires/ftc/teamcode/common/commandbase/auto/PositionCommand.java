package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

import java.util.Locale;

public class PositionCommand extends CommandBase {
    public final double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public final double ALLOWED_HEADING_ERROR = Math.toRadians(1);

    public final PIDFController mController = new PIDFController(0.03, 0, 0.5, 0);
    public final PIDFController hController = new PIDFController(0.4, 0, 0.2, 0);

    SwerveDrivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;
    ElapsedTime deadTimer;

    private final double ms;
    private final double delay;
    private ElapsedTime delayTimer;

    private final double v;

    public PositionCommand(SwerveDrivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double dead, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = dead;
        this.delay = delay;
        this.v = voltage;
    }

    public PositionCommand(SwerveDrivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = Integer.MAX_VALUE;
        this.delay = delay;
        this.v = voltage;
    }

    @Override
    public void initialize() {
        Globals.USE_WHEEL_FEEDFORWARD = true;
    }

    @Override
    public void execute() {
        if (deadTimer == null) {
            deadTimer = new ElapsedTime();
        }

        Pose powers = goToPosition(localizer.getPos(), targetPose);
        System.out.println(powers.toString());
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
        Globals.USE_WHEEL_FEEDFORWARD = false;

    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = targetPose.subtract(robotPose);

        deltaPose.x = MathUtils.clamp(deltaPose.x, -20, 20);
        deltaPose.y = MathUtils.clamp(deltaPose.y, -20, 20);

        double magnitude = hypot(deltaPose.y, deltaPose.x);
        double dir = normalizeRadians(atan2(deltaPose.y, deltaPose.x) - robotPose.heading);

        double power = mController.calculate(0, magnitude);

        // I hate my life
        double correction = (0.055 * (v-12.4));
        dir += (correction / 3.3 * 2 * Math.PI * MathUtils.clamp(power, 0, 1));

        if (Math.abs(power) < 0.01) power = 0;

        double y_component = cos(dir) * power;
        double x_component = sin(dir) * power;
        double heading_component = hController.calculate(0, deltaPose.heading);

        System.out.printf(Locale.ENGLISH, "dir: %.3f, y: %.3f, x: %.3f", dir, y_component, x_component);


        if (Math.abs(heading_component) < 0.015) heading_component = 0;

        double max = 1;
        Pose powers = new Pose(
                MathUtils.clamp(x_component / v * 12, -max, max),
                MathUtils.clamp(-y_component / v * 12, -max, max),
                -heading_component / v * 12
        );

        return new Pose(powers.x, powers.y, powers.heading);
    }
}
