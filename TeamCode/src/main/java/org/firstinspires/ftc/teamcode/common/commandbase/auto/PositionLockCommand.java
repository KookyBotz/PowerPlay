package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

import java.util.function.BooleanSupplier;

public class PositionLockCommand extends CommandBase {
    public final double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public final double ALLOWED_HEADING_ERROR = Math.toRadians(2);

    public final PIDFController hController = new PIDFController(0.5, 0, 0.25, 0);
    public final PIDFController mController = new PIDFController(0.3, 0, 0.2, 0);

    private final SwerveDrivetrain drivetrain;
    private final Localizer localizer;
    private static Pose targetPose;

    private final BooleanSupplier endSupplier;

    private ElapsedTime lockedTimer;
    private boolean reached;

    private final double v;

    public PositionLockCommand(SwerveDrivetrain drivetrain, Localizer localizer, BooleanSupplier end, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.v = voltage;
        this.endSupplier = end;
    }

    public static void setTargetPose(Pose target) {
        PositionLockCommand.targetPose = target;
    }

    public static Pose getTargetPose() {
        return targetPose;
    }

    @Override
    public void initialize() {
        Globals.USE_WHEEL_FEEDFORWARD = true;
        lockedTimer = new ElapsedTime();
    }

    @Override
    public void execute() {
        if (targetPose.x == 0 && targetPose.y == 0 && targetPose.heading == 0) {
            drivetrain.setLocked(false);
            return;
        }

        Pose powers = goToPosition(localizer.getPos(), targetPose);

        if (!reached) lockedTimer.reset();

        drivetrain.setLocked(lockedTimer.milliseconds() > 500);
        drivetrain.set(reached ? new Pose() : powers);
    }

    @Override
    public boolean isFinished() {
        return endSupplier.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
        Globals.USE_WHEEL_FEEDFORWARD = false;
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = targetPose.subtract(robotPose);

        reached = Math.hypot(deltaPose.x, deltaPose.y) < ALLOWED_TRANSLATIONAL_ERROR && Math.abs(deltaPose.heading) < ALLOWED_HEADING_ERROR;

        double x_rotated = deltaPose.x * cos(robotPose.heading) - deltaPose.y * sin(robotPose.heading);
        double y_rotated = deltaPose.x * sin(robotPose.heading) + deltaPose.y * cos(robotPose.heading);

        double magnitude = hypot(y_rotated, x_rotated);
        double dir = atan2(y_rotated, x_rotated);
        double power = mController.calculate(0, magnitude);

        double y_component = cos(dir) * power;
        double x_component = sin(dir) * power;
        double heading_component = hController.calculate(0, deltaPose.heading);

        Pose powers = new Pose(
                x_component / v * 12.5,
                -y_component / v * 12.5,
                -heading_component / v * 12.5
        );

        return new Pose(powers.x, powers.y, powers.heading);
    }
}
