package org.firstinspires.ftc.teamcode.common.purepursuit.path;

import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.MAX_SPEED;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitConfig.pCoefficientY;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.Arrays;
import java.util.List;


public class PurePursuitPath {
    private final List<Waypoint> waypoints;
    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final boolean pController;

    private int currentWaypoint = 0;
    private ElapsedTime timer;
    private MotionProfile profile;

    public PurePursuitPath(Drivetrain drivetrain, Localizer localizer, boolean pController, MotionProfile profile, Waypoint... waypoints) {
        this.waypoints = Arrays.asList(waypoints);
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.pController = pController;
        this.profile = profile;

        if (!(waypoints[waypoints.length - 1].getPos() instanceof Pose)) {
            throw new IllegalArgumentException("Last Waypoint Must Be a Pose");
        }
    }

    public PurePursuitPath(Drivetrain drivetrain, Localizer localizer, Waypoint... waypoints) {
        this(drivetrain, localizer, true, new RisingMotionProfile(Integer.MAX_VALUE, 1), waypoints);
    }

    public PurePursuitPath(Drivetrain drivetrain, Localizer localizer, boolean pController, Waypoint... waypoints) {
        this(drivetrain, localizer, pController, new RisingMotionProfile(Integer.MAX_VALUE, 1), waypoints);
    }

    // returns false if done, return true otherwise
    public boolean update() {
        if (timer == null) timer = new ElapsedTime();


        Pose robotPose = localizer.getPos();
        Waypoint nextWaypoint = waypoints.get(currentWaypoint + 1);
        Point previousPoint = waypoints.get(currentWaypoint).getPos();
        Point nextPoint = waypoints.get(currentWaypoint + 1).getPos();

        // check if we are done with our path (reached last point)
        if (currentWaypoint == waypoints.size() - 2 && robotPose.distanceTo(nextPoint) < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR) {
            drivetrain.set(new Pose(0, 0, 0));
            return false;
        }


        if (currentWaypoint == waypoints.size() - 2 && robotPose.distanceTo(nextPoint) < nextWaypoint.getFollowDistance()) {
            Pose powers = PurePursuitController.goToPosition(robotPose, (Pose) nextPoint, new Pose(
                    pCoefficientX, pCoefficientY, pCoefficientH
            ));

            drivetrain.set(pController ? powers : new Pose());
            return true;
        }


        // check if we reached the target waypoint, if so increment and move on
        if (robotPose.distanceTo(nextPoint) < nextWaypoint.getFollowDistance() && nextWaypoint.getFollowDistance() != 0) {
            System.out.println("heading error " + Math.abs(AngleUnit.normalizeRadians(robotPose.heading - ((Pose) nextPoint).heading)));
            System.out.println(currentWaypoint);
            currentWaypoint++;
            return true;
        }

        if (nextWaypoint.getFollowDistance() == 0 && robotPose.distanceTo(nextPoint) < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(AngleUnit.normalizeRadians(robotPose.heading - ((Pose) nextPoint).heading)) < PurePursuitConfig.ALLOWED_HEADING_ERROR) {
            System.out.println(currentWaypoint);
            System.out.println("next point");
            currentWaypoint++;
            return true;
        }

        Pose intersection;


        // Calculate intersection point
        Point intersectionPoint = PurePursuitController.lineCircleIntersection(
                previousPoint, nextPoint, robotPose, nextWaypoint.getFollowDistance()
        );

        if (nextWaypoint.getFollowDistance() == 0) {
            Pose powers = PurePursuitController.goToPosition(robotPose, (Pose) nextPoint, new Pose(
                    pCoefficientX, pCoefficientY, pCoefficientH
            ));

            drivetrain.set(powers, nextWaypoint.maxPower * profile.update(timer.seconds()));

            return true;
        }

        // we want to hold heading
        if (nextPoint instanceof Pose) {

            intersection = new Pose(intersectionPoint.x, intersectionPoint.y, ((Pose) nextPoint).heading);
        }

        // we want heading to be tangent to line
        else {

            double heading = intersectionPoint.subtract(robotPose).atan();
            double reverseHeading = AngleUnit.normalizeRadians(heading + Math.PI);

            double autoAngle = AngleUnit.normalizeRadians(robotPose.heading - heading) <
                    AngleUnit.normalizeRadians(robotPose.heading - reverseHeading) ?
                    heading : reverseHeading;

            intersection = new Pose(intersectionPoint.x, intersectionPoint.y, autoAngle);
        }


        // Calculate Powers
        Pose powers = PurePursuitController.goToPosition(robotPose, intersection, new Pose(
                pCoefficientX, pCoefficientY, pCoefficientH
        ));


        // Set Powers
        drivetrain.set(powers, profile.update(timer.seconds()) / MAX_SPEED);
        return true;
    }
}
