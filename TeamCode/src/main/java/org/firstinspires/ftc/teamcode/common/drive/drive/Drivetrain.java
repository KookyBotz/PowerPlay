package org.firstinspires.ftc.teamcode.common.drive.drive;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

public interface Drivetrain {

    void set(Pose pose);

    void set(Pose pose, double maxPower);
}
