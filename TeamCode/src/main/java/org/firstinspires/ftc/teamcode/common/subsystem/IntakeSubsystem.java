package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx extension;
    private final Servo barLeft, barRight;
    private final Servo claw, turret;

    public static int extension_out_pos = 100;
    public static int extension_in_pos = 0;

    public static double claw_open_pos = 10;
    public static double claw_closed_pos = 5;

    public static double forebar_extended = 0.4;
    public static double forebar_retracted = 0.1;

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

    public void extensionOut() {
        extension.setTargetPosition(extension_out_pos);
    }

    public void extensionIn() {
        extension.setTargetPosition(extension_in_pos);
    }

    public void closeClaw() {
        claw.setPosition(claw_closed_pos);
    }

    public void openClaw() {
        claw.setPosition(claw_open_pos);
    }

    public void extendForebar() {
        barLeft.setPosition(forebar_extended);
        barRight.setPosition(forebar_extended);
    }

    public void closedForebar() {
        barLeft.setPosition(forebar_retracted);
        barRight.setPosition(forebar_retracted);
    }
}
