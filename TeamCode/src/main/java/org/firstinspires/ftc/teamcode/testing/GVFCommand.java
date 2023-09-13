package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class GVFCommand extends CommandBase {

    GVFController controller;
    Localizer localizer;
    HermitePath path;
    Drivetrain drivetrain;

    public GVFCommand(Drivetrain drivetrain, Localizer localizer, HermitePath path) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.path = path;
        this.controller = new GVFController(path, localizer.getPos());
    }

    @Override
    public void initialize() {
        Globals.USE_WHEEL_FEEDFORWARD = true;
    }

    @Override
    public void execute() {
        controller.setCurrentPose(localizer.getPos());
        Pose gvfPose = controller.calculateGVF();

        // use components to generate the power required lol
    }

    @Override
    public boolean isFinished() {
        return controller.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Globals.USE_WHEEL_FEEDFORWARD = true;
    }
}
