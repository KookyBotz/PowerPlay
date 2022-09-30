package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx extension;
    private final Servo barLeft, barRight;

    private final Servo claw, turret;

    public IntakeSubsystem(MotorEx extension, Servo barLeft, Servo barRight, Servo claw, Servo turret) {
        this.extension = extension;
        this.barLeft = barLeft;
        this.barRight = barRight;

        this.claw = claw;
        this.turret = turret;
    }

    public void setExtension(int pos) {
        extension.setTargetPosition(pos);
    }
}
