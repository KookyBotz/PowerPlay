package org.firstinspires.ftc.teamcode.common.purepursuit.localizer;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
