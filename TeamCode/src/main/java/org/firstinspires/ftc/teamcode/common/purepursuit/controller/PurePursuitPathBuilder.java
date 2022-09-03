package org.firstinspires.ftc.teamcode.common.purepursuit.controller;


import org.firstinspires.ftc.teamcode.common.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitPathBuilder {
    private Localizer localizer;
    private Drivetrain drivetrain;
    private double currentPower = 1;
    private double followDistance = 10;
    private List<Waypoint> waypoints;

    public PurePursuitPathBuilder() {
        waypoints = new ArrayList<>();
    }

    public PurePursuitPathBuilder setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        return this;
    }

    public PurePursuitPathBuilder setLocalizer(Localizer localizer) {
        if (drivetrain == null) throw new IllegalStateException("set drivetrain first please");
        this.localizer = localizer;

        return this;
    }

    public PurePursuitPathBuilder setPower(double power) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first please");

        this.currentPower = power;
        return this;
    }

    public PurePursuitPathBuilder setFollowDistance(double distance) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first please");

        this.followDistance = distance;
        return this;
    }

    public PurePursuitPathBuilder then(Pose pose) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first please");
        waypoints.add(new Waypoint(pose, followDistance, currentPower));

        return this;
    }

    public PurePursuitPath build() {
        if (drivetrain == null || localizer == null || waypoints.size() <= 2)
            throw new IllegalStateException("missing some info sad");
        return new PurePursuitPath(drivetrain, localizer, waypoints.toArray(new Waypoint[0]));
    }
}
