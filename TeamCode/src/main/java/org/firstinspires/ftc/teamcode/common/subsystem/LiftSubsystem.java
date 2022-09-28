package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubsystem extends SubsystemBase {
    public final DcMotorEx lift;

    private final PIDController controller;


    public LiftSubsystem(DcMotorEx lift) {
        this.lift = lift;
    }
}
