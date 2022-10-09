package org.firstinspires.ftc.teamcode.common.purepursuit.path;


import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.RisingMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitPathBuilder {
    private Localizer localizer;
    private Drivetrain drivetrain;
    private MotionProfile profile = new RisingMotionProfile(Integer.MAX_VALUE, 1);
    private double followDistance = 10;
    private List<Waypoint> waypoints;
    private boolean pController = true;

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

    public PurePursuitPathBuilder setStartPosition(Pose pose) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer cringe");
        this.localizer.setPos(pose);
        return this;
    }

    public PurePursuitPathBuilder setController(boolean pController){
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first please");

        this.pController = pController;
        return this;
    }

    public PurePursuitPathBuilder setFollowDistance(double distance) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first please");

        this.followDistance = distance;
        return this;
    }

    public PurePursuitPathBuilder setMotionProfile(MotionProfile profile) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first PLEASE");

        this.profile = profile;
        return this;
    }

    public PurePursuitPathBuilder then(Pose pose) {
        if (drivetrain == null || localizer == null)
            throw new IllegalStateException("set drivetrain and localizer first please");
        waypoints.add(new Waypoint(pose, followDistance, 1.0));

        return this;
    }

    public PurePursuitPath build() {
        if (drivetrain == null || localizer == null || waypoints.size() <= 2)
            throw new IllegalStateException("missing some info sad");
        return new PurePursuitPath(drivetrain, localizer, pController, profile, waypoints.toArray(new Waypoint[0]));
    }


}
