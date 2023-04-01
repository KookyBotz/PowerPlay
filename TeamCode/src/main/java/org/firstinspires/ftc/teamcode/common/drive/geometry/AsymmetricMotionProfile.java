package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class AsymmetricMotionProfile {
    public double initialPosition;
    public double finalPosition;
    public double distance;
    public double t1, t2, t3;
    public double totalTime;
    public double accelStopPosition;
    public double accelStopVelocity;
    public boolean flipped = false;
    public double originalPos = 0;

    public State state = new State();
    public Constraints constraints;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, Constraints constraints) {
        if (finalPosition < initialPosition) {
            flipped = true;
            this.originalPos = initialPosition;
            double temp = initialPosition;
            initialPosition = finalPosition;
            finalPosition = temp;
        }
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.distance = finalPosition - initialPosition;
        this.constraints = constraints;

        t1 = constraints.velo / constraints.accel;
        t3 = constraints.velo / constraints.decel;

        t2 = Math.abs(distance) / constraints.velo - (t1 + t3) / 2;

        if (t2 < 0) {
//            System.out.println("Non-Max Velocity Profile");
            this.t2 = 0;

            // this math utilizes a negative deceleration constant. either negate from the passed in value,
            // or just add a negatation symbol prior to the variable.
            double a = (constraints.accel / 2) * (1 - constraints.accel / -constraints.decel);
            // System.out.println("a " + a);
            double c = -distance;
            // System.out.println("c " + c);

            // acceleration phase
            t1 = Math.sqrt(-4 * a * c) / (2 * a);
            // System.out.println("t1 " + t1);

            // empty phase
            // System.out.println("t2 " + t2);

            // deceleration phase
            t3 = -(constraints.accel * t1) / -constraints.decel;
            // System.out.println("t3 " + t3);

            // ending accel position (middle peak)
            accelStopPosition = (constraints.accel * Math.pow(t1, 2)) / 2;
            // System.out.println("xA " + accelStopPosition);

            // ending accel velocity (middle peak)
            accelStopVelocity = constraints.accel * t1;
            // System.out.println("vA " + accelStopVelocity);
        } else {
            System.out.println("Max Velocity Profile");
            // time constants already calculated

            accelStopVelocity = constraints.velo;
            accelStopPosition = (constraints.velo * t1) / 2;
        }

        totalTime = t1 + t2 + t3;

        log();
    }

    public State calculate(final double time) {
        double position = 0;
        double velocity = 0;
        double acceleration = 0;
        if (time <= t1) {
            acceleration = constraints.accel;
            velocity = time * constraints.accel;
            position = (constraints.accel * Math.pow(time, 2)) / 2;
        } else if (time <= t1 + t2) {
            acceleration = 0;
            velocity = constraints.velo;
            position = accelStopPosition + (time - t1) * constraints.velo;
        } else if (time <= totalTime) {
            acceleration = -constraints.decel;
            velocity = accelStopVelocity - (time - t1 - t2) * constraints.decel;
            position = accelStopPosition + (t1 * constraints.accel) * (time - t1) - (constraints.decel * Math.pow(time - t1 - t2, 2)) / 2;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        // state.x = (finalPosition > initialPosition) ? initialPosition + position : initialPosition - position;
        if (flipped) {
            state.x = originalPos - position;
        } else {
            state.x = initialPosition + position;
        }
        state.v = velocity;
        state.a = acceleration;
        return this.state;
    }

    public void log() {

    }
}