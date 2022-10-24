package org.firstinspires.ftc.teamcode.common.purepursuit.drive;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public interface Drivetrain {

    void set(Pose pose);

    void set(Pose pose, double maxPower);
}
