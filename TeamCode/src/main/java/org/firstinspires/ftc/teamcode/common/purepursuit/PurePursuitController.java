package org.firstinspires.ftc.teamcode.common.purepursuit;


public class PurePursuitController {

    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }


    public static Pose goToPosition(Pose robotPose, Pose targetPose, Pose pCoefficients) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
        Pose powers = deltaPose.divide(pCoefficients);
        double x_rotated = powers.x * Math.cos(-robotPose.heading) - powers.y * Math.sin(-robotPose.heading);
        double y_rotated = powers.x * Math.sin(-robotPose.heading) + powers.y * Math.cos(-robotPose.heading);
        return new Pose(x_rotated, y_rotated, powers.heading);
    }
}
