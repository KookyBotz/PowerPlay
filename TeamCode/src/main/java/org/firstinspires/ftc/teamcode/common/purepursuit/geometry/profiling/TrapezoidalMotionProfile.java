package org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling;

public class TrapezoidalMotionProfile implements MotionProfile {
    private final double maxV;
    private final double maxA;
    private final double distance;

    private final double inverseTime;
    private double dRad;
    private double tRad;
    private double tCir;
    private double dCir;
    private double aCur;
    private double velocity;

    public TrapezoidalMotionProfile(double maxV, double maxA, double distance) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.distance = distance;

        this.inverseTime = Math.pow(maxV, 2) / Math.pow(maxA, 3);
    }

    @Override
    public double update(double time)
    {
        if (time < 0) {
            return 0;
        }

        if (inverseTime <= distance) {
            tRad = maxV / maxA;
            dRad = (maxA * Math.pow(tRad, 2)) / 2;
        } else {
            tRad = Math.sqrt(distance / maxA);
            dRad = distance / 2;
        }

        dCir = distance - (2 * dRad);
        tCir = dCir / maxV;

        aCur = getAccel(time);

        if (time <= tRad) {
            velocity =  aCur * time;
        } else if ((time - tRad) <= tCir) {
            velocity = maxV;
        } else if ((time - tRad - tCir) <= tRad) {
            velocity = getAccel(tRad) * tRad - maxA * (time - tCir - tRad);
        }

        return velocity;
    }

    private double getAccel(double time) {
        if (time <= tRad) {
            return maxA;
        } else if (time <= (tRad + tCir)) {
            return 0.0;
        } else {
            return (2 * tRad) + tCir;
        }
    }
}
