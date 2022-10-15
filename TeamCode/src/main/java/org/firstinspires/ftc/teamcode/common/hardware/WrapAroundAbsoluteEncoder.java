package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Kinematics;

public class WrapAroundAbsoluteEncoder extends AbsoluteAnalogEncoder {
    private boolean leftHalf = true;
    private double tolerance = Math.PI / 20;

    public WrapAroundAbsoluteEncoder(AnalogInput enc) {
        super(enc);
    }

    private double pastPosition = 1;
    @Override
    public double getCurrentPosition() {
        // encoder get voltage
        // returns 0-3.3v
        // keep track of what last reading was
        // if reading jumps from 3.3v to 0 going in one direction
        // if reading jumps from 0 to 3.3v then going in other direction
        // 0-3.3v to pi
        // if right half then
        // 0-3.3v to pi - 2pi
//        double min = (leftHalf) ? 0 : Math.PI;
//        double currentPosition = map(super.getEncoder().getVoltage(), 0, 3.3, 0, Math.PI);
        double currentPosition = Kinematics.map(super.getEncoder().getVoltage(), 0, 3.3, 0, Math.PI);
        if ((pastPosition >= Math.PI - tolerance && currentPosition <= 0 + tolerance) ||
           (pastPosition <= 0 + tolerance && currentPosition >= Math.PI - tolerance)){
            leftHalf = !leftHalf;
        }

        pastPosition = currentPosition;
        return (leftHalf) ? currentPosition : currentPosition + Math.PI;
    }

    private double map(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
