package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;

public class Waypoint {

    private final Point pos;

    public double maxPower = -1;
    private final double followDistance;

    public Waypoint(Point pos, double followDistance) {
        this.pos = pos;

        this.followDistance = followDistance;
    }

    public Waypoint(Point pos, double followDistance, double maxPower) {
        this(pos, followDistance);
        this.maxPower = maxPower;
    }

    public Point getPos(){
        return pos;
    }

    public double getFollowDistance() {
        return followDistance;
    }
}
