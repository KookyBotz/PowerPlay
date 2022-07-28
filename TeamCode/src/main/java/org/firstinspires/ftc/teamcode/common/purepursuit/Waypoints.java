package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.arcrobotics.ftclib.command.Command;

public class Waypoints {

    enum Reversed {
        FORWARD,
        BACKWARD,
        DEFAULT
    };

    private Point pos;
    private Reversed driveDir;

    private Command onStart;
    private Command onEnd;

    private double velocity;

    public Waypoints(Point pos, Reversed driveDir, Command onStart, Command onEnd, double velocity) {
        this.pos = pos;
        this.driveDir = driveDir;

        this.onStart = onStart;
        this.onEnd = onEnd;

        this.velocity = velocity;
    }
}
