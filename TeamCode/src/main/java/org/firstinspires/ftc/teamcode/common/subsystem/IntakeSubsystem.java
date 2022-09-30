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

    public static int claw_open_pos = 10;
    public static int claw_closed_pos = 5;

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

    public void closeClaw() {
        claw.setPosition(claw_closed_pos);
    }

    public void openClaw() {
        claw.setPosition(claw_open_pos);
    }
}
