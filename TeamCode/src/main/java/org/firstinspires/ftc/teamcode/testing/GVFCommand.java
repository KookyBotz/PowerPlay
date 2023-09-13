package org.firstinspires.ftc.teamcode.testing;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class GVFCommand extends CommandBase {

    GVFController controller;
    Localizer localizer;
    HermitePath path;
    Drivetrain drivetrain;

    public static double xP = 0.0;
    public static double xD = 0.0;
    public static double xF = 0;

    public static double yP = 0.0;
    public static double yD = 0.0;
    public static double yF = 0;

    public static double hP = 0.6;
    public static double hD = 0.3;
    public static double hF = 0;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);
    public static double max_power = 1;
    public static double max_heading = 0.5;

    private final double voltage;

    public GVFCommand(Drivetrain drivetrain, Localizer localizer, HermitePath path, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.path = path;
        this.controller = new GVFController(path, localizer.getPos());
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        Globals.USE_WHEEL_FEEDFORWARD = true;
    }

    @Override
    public void execute() {
        Pose robotPose = localizer.getPos();
        controller.setCurrentPose(robotPose);
        Pose gvf = controller.calculateGVF();
        Pose powers = getPowerS(gvf, robotPose);
        controller.powers = powers;
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        return controller.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Globals.USE_WHEEL_FEEDFORWARD = true;
    }

    public Pose getPowerS(Pose gvf, Pose robotPose) {
        Pose powers = new Pose(
                xController.calculate(0, gvf.x),
                yController.calculate(0, gvf.y),
                hController.calculate(0, gvf.heading)
        );
        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
        double x_power = -x_rotated < -max_power ? -max_power :
                Math.min(-x_rotated, max_power);
        double y_power = -y_rotated < -max_power ? -max_power :
                Math.min(-y_rotated, max_power);
        double heading_power = MathUtils.clamp(powers.heading, -max_heading, max_heading);

        if(Math.abs(x_power) < 0.01) x_power = 0;
        if(Math.abs(y_power) < 0.01) y_power = 0;

        return new Pose(-y_power / voltage * 12, x_power / voltage * 12, -heading_power / voltage * 12);
    }
}
