package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;

@Config
public class LiftSubsystem extends SubsystemBase {
    public final DcMotorEx lift;

    private final PIDController controller;
    private final MotionProfile profile;

    public static double maxA = 1;
    public static double maxV = 2;

    public LiftSubsystem(DcMotorEx lift) {
        this.lift = lift;


    }
}
