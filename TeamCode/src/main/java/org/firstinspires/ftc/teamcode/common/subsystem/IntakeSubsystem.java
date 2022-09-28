package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final Servo barLeft, barRight;

    public IntakeSubsystem(Servo barLeft, Servo barRight) {
        this.barLeft = barLeft;
        this.barRight = barRight;
    }

    public void setPos(double pos) {

    }
}
