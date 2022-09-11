package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientY;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitController;

public class PositionCommand extends CommandBase {
    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
    }

    @Override
    public void execute() {
        drivetrain.set(PurePursuitController.goToPosition(localizer.getPos(), targetPose, new Pose(pCoefficientX, pCoefficientY, pCoefficientH)));
    }

    @Override
    public boolean isFinished() {
        Pose error = targetPose.subtract(localizer.getPos());
        if (error.x < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR && error.y < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR && error.heading < PurePursuitConfig.ALLOWED_HEADING_ERROR) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}
