package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

import java.util.Locale;

@Config
public class PositionLock {
    public static double hP = 0.4;
    public static double hD = 0.2;
    public static double hI = 0;

    public static double mP = 0.15;
    public static double mD = 0.0;
    public static double mI = 0.0;

    public static PIDFController hController = new PIDFController(hP, hI, hD, 0);
    public static PIDFController mController = new PIDFController(mP, mI, mD, 0);

    public static Pose calculate(Pose robotPose, Pose targetPose, double voltage) {
        Pose deltaPose = targetPose.subtract(robotPose);

        double x_rotated = deltaPose.x * cos(robotPose.heading) - deltaPose.y * sin(robotPose.heading);
        double y_rotated = deltaPose.x * sin(robotPose.heading) + deltaPose.y * cos(robotPose.heading);

        double magnitude = hypot(y_rotated, x_rotated);
        double dir = atan2(y_rotated, x_rotated);

        double power = mController.calculate(0, magnitude);

        double y_component = cos(dir) * power;
        double x_component = sin(dir) * power;

//        System.out.printf(Locale.ENGLISH, "mag: %.2f, dir: %.2f, pow: %.2f, x: %.2f, y: %.2f%n", magnitude, dir, power, x_component, y_component);

        Pose powers = new Pose(
                x_component / voltage * 12.5,
                -y_component / voltage * 12.5,
                -hController.calculate(0, deltaPose.heading) / voltage * 12.5
        );

        return new Pose(powers.x, powers.y, powers.heading);
    }
}