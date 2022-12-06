package org.firstinspires.ftc.teamcode.common.drive.localizer;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class BetterSwerveLocalizer implements Localizer, com.acmerobotics.roadrunner.localization.Localizer {
    public SwerveModule.SwerveModuleState[] modules;
    public DoubleSupplier imu;
    public Pose2d poseEstimate;
    public Pose2d pastPoseEstimate;
    public BetterSwerveLocalizer(DoubleSupplier i, SwerveModule... mods){
        modules = Arrays.stream(mods).map(SwerveModule::asState).toArray(SwerveModule.SwerveModuleState[]::new);
        imu = i;
        poseEstimate = new Pose2d();
        pastPoseEstimate = new Pose2d();
    }
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseEstimate.minus(pastPoseEstimate);
    }

    @Override
    public void periodic() {
        update();
    }


    @Override
    public Pose getPos() {
        Pose2d pose = getPoseEstimate();
        return new Pose(pose.getY(), pose.getX(), -pose.getHeading());
    }

    @Override
    public void setPos(Pose pose) {
        setPoseEstimate(new Pose2d(pose.y, pose.x, pose.heading));
    }

    @Override
    public void update() {
        pastPoseEstimate = poseEstimate;
        Vector2d accumulator = new Vector2d();
        double head = imu.getAsDouble();
        for(SwerveModule.SwerveModuleState s : modules){
            accumulator = accumulator.plus(s.calculateDelta());
        }
        accumulator = accumulator.div(modules.length).rotated(head);
        poseEstimate = new Pose2d(poseEstimate.vec().plus(accumulator), head);
    }

}
