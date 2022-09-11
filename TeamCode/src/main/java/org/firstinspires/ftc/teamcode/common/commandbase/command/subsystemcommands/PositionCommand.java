package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientY;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;

public class PositionCommand extends CommandBase {
    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
    }

/*
drive = PurePursuitController.goToPosition(
                        localizer.getPos(), targetPose, new Pose(pCoefficientX, pCoefficientY, pCoefficientH)
                );
 */
}
