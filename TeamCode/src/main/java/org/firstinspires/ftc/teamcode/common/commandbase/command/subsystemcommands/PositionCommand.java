package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientY;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitController;

public class PositionCommand extends CommandBase {
    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;
    MotionProfile profile;
    ElapsedTime timer;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.profile = profile;
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
        }
        drivetrain.set(PurePursuitController.goToPosition(localizer.getPos(), targetPose, new Pose(pCoefficientX, pCoefficientY, pCoefficientH)));
    }

    @Override
    public boolean isFinished() {
        double heading_error = Math.abs(AngleUnit.normalizeRadians(targetPose.heading - localizer.getPos().heading));
        boolean a = (Math.abs(localizer.getPos().distanceTo(targetPose)) < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && heading_error < PurePursuitConfig.ALLOWED_HEADING_ERROR);
        System.out.print("HERE123" + a);
        return a;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
        System.out.println("false");
    }
}
