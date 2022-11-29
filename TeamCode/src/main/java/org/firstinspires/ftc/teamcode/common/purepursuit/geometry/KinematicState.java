package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import com.qualcomm.robotcore.util.Range;

public class KinematicState {
    public final int intakeStartingPos;
    public final int intakeEndPos;

    public final double intakeVelo;
    public final double intakeAccel;

    public final double fourbarStartPos;
    public final double fourbarEndPos;

    public double time = 0;

    private final double DEGREES_PER_TICK = 360 / 2.04;
    private final double ROTATIONS_PER_SECOND = 1.76;
    private final double TICKS_PER_INCH = 23.5;
    private final double C2C_DISTANCE = 225 / 25.4;

    public KinematicState(int intakeStartingPos, double fourbarStartPos, double fourbarEndPos) {
        this.intakeStartingPos = intakeStartingPos;

        this.fourbarStartPos = fourbarStartPos;
        this.fourbarEndPos = fourbarEndPos;

        double fourbarStartPosRad = Math.toRadians((fourbarStartPos - 0.24) * DEGREES_PER_TICK);
        double fourbarEndPosRad = Math.toRadians((fourbarEndPos - 0.24) * DEGREES_PER_TICK);

//        double deltaTheta = DEGREES_PER_TICK * (this.fourbarEndPos - this.fourbarStartPos);
//        double time = deltaTheta / ROTATIONS_PER_SECOND;

        double fourbarStartX = Math.cos(fourbarStartPosRad) * C2C_DISTANCE;
        double fourbarEndX = Math.cos(fourbarEndPosRad) * C2C_DISTANCE;

        int deltaPosition = (int) (Math.abs(fourbarEndX - fourbarStartX) * TICKS_PER_INCH);

        this.intakeEndPos = Range.clip(this.intakeStartingPos + deltaPosition, 0, 575);

//        this.intakeVelo = (this.intakeEndPos - this.intakeStartingPos) / time;
        this.intakeVelo = 500;
//        this.intakeAccel = intakeVelo / time;
        this.intakeAccel = 1000;

        System.out.println("end pos: " + intakeEndPos);
    }

    public void update(Pose robotPose, Pose targetPose) {

        // CONE POSE: -35.25, 51.63
    }
}