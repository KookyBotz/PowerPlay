package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;

public class Waypoint {

    private final Point pos;

    private final double followDistance;

    public Waypoint(Point pos, double followDistance) {
        this.pos = pos;

        this.followDistance = followDistance;
    }

    public Point getPos(){
        return pos;
    }

    public double getFollowDistance() {
        return followDistance;
    }
}
