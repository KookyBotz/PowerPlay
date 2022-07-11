package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
    private final Servo left;
    private final Servo right;

    public TurretSubsystem(Servo left, Servo right) {
        this.left = left;
        this.right = right;
    }
}
