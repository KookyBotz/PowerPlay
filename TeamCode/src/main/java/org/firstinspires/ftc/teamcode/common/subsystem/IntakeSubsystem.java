package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx extension;
    private final Servo barLeft, barRight;

    public IntakeSubsystem(MotorEx extension, Servo barLeft, Servo barRight) {
        this.extension = extension;
        this.barLeft = barLeft;
        this.barRight = barRight;
    }

    public void setExtension(int pos) {
        extension.setTargetPosition(pos);
    }
}
