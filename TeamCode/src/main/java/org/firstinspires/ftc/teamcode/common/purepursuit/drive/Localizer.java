package org.firstinspires.ftc.teamcode.common.purepursuit.drive;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();
}
