package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class GrabPosition {
    public long clawDelay, turretDelay;
    public double fourbarPos, pivotPos;
    public double   intakeTargetPosition;

    public GrabPosition(int pos, long clawDelay, double fourbarPos, double pivotPos, long upDelay) {
        this.clawDelay = clawDelay;
        this.turretDelay = upDelay;
        this.pivotPos = pivotPos;
        this.fourbarPos = fourbarPos;
        this.intakeTargetPosition = pos;
    }
}