package org.firstinspires.ftc.teamcode.common.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
//BRUH JUST DONT USE ROADRUNNER
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer implements Localizer {
    public static double TICKS_PER_REV = 8092;
    public static double WHEEL_RADIUS = 0.689; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    // -1.425
    public static double PARALLEL_X = -4.125; // X is the up and down direction
    public static double PARALLEL_Y = -4.125; // Y is the strafe direction
    // -1.375
    public static double PERPENDICULAR_X = -1.375;
    public static double PERPENDICULAR_Y = -1.375;

    private final DoubleSupplier horizontalPosition, lateralPosition, imuAngle;

    public TwoWheelLocalizer(DoubleSupplier horizontalPosition, DoubleSupplier lateralPosition, DoubleSupplier imuAngle) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.horizontalPosition = horizontalPosition;
        this.lateralPosition = lateralPosition;
        this.imuAngle = imuAngle;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imuAngle.getAsDouble();
    }

    @Override
    public Double getHeadingVelocity() {
        return 0.0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(horizontalPosition.getAsDouble()),
                encoderTicksToInches(lateralPosition.getAsDouble())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(0.0, 0.0);
    }

    @Override
    public Pose getPos() {
        Pose2d pose = getPoseEstimate();
        return new Pose(-pose.getX(), pose.getY(), pose.getHeading());
    }

    @Override
    public void setPos(Pose pose) {
        ;
    }

    @Override
    public void periodic() {
        super.update();
    }
}