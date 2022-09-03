package org.firstinspires.ftc.teamcode.common.purepursuit.controller;

import static org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitConfig.pCoefficientH;
import static org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitConfig.pCoefficientX;
import static org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitConfig.pCoefficientY;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;


public class PurePursuitPath {
    private final List<Waypoint> waypoints;
    private final Drivetrain drivetrain;
    private final Localizer localizer;

    private int currentWaypoint = 0;

    public PurePursuitPath(Drivetrain drivetrain, Localizer localizer, Waypoint... waypoints) {
        this.waypoints = Arrays.asList(waypoints);
        this.drivetrain = drivetrain;
        this.localizer = localizer;

        if (!(waypoints[waypoints.length - 1].getPos() instanceof Pose)) {
            throw new IllegalArgumentException("Last Waypoint Must Be a Pose");
        }
    }

    // returns false if done, return true otherwise
    public boolean update() {

        // check if we are done with our path (reached last point)
        if (currentWaypoint == waypoints.size() - 1) {
            drivetrain.set(new Pose(0, 0, 0));
            return false;
        }

        Pose robotPose = localizer.getPos();
        Waypoint nextWaypoint = waypoints.get(currentWaypoint + 1);
        Point previousPoint = waypoints.get(currentWaypoint).getPos();
        Point nextPoint = waypoints.get(currentWaypoint + 1).getPos();

        if (currentWaypoint == waypoints.size() - 2 && robotPose.distanceTo(nextPoint) < nextWaypoint.getFollowDistance()) {
            Pose powers = PurePursuitController.goToPosition(robotPose, (Pose) nextPoint, new Pose(
                    pCoefficientX, pCoefficientY, pCoefficientH
            ));

            drivetrain.set(powers);
            return true;
        }


        // check if we reached the target waypoint, if so increment and move on
        if (robotPose.distanceTo(nextPoint) < nextWaypoint.getFollowDistance()) {
            currentWaypoint++;
            return true;
        }

        Pose intersection;


        // Calculate intersection point
        Point intersectionPoint = PurePursuitController.lineCircleIntersection(
                previousPoint, nextPoint, robotPose, nextWaypoint.getFollowDistance()
        );


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
        drivetrain.set(powers, nextWaypoint.maxPower);
        return true;
    }
}
