package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx extension;
    private final Servo barLeft, barRight;
    private final Servo claw, turret;

    private MotionProfile profile;
    private final ElapsedTime timer;

    public static int intake_out_pos = 100;
    public static int intake_in_pos = 0;

    public static double claw_open_pos = 10;
    public static double claw_closed_pos = 5;

    public static double forebar_extended = 0.4;
    public static double forebar_middle = 0.25;
    public static double forebar_retracted = 0.1;

    public static final double FOREBAR_LENGTH = 9.842;

    public IntakeSubsystem(MotorEx extension, Servo barLeft, Servo barRight, Servo claw, Servo turret, MotionProfile profile) {
        this.extension = extension;
        this.barLeft = barLeft;
        this.barRight = barRight;
        this.claw = claw;
        this.turret = turret;

        this.profile = profile;
        this.timer = new ElapsedTime();
    }

    public void loop() {
        double target = profile.update(timer.time())[1];
        extension.setTargetPosition((int) target);
        // pid control
    }

    public void setMotionProfile(MotionProfile profile) {
        this.profile = profile;
    }

    public void setExtension(int pos) {
        extension.setTargetPosition(pos);
    }

    public void setForebar(double pos) {
        // TODO set forebar
    }

    public void setClaw(double position) {
        claw.setPosition(position);
    }

    public int getExtension() {
        return extension.getCurrentPosition();
    }

    public void extensionOut() {
        extension.setTargetPosition(intake_out_pos);
    }

    public void extensionIn() {
        extension.setTargetPosition(intake_in_pos);
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

    public void closeForebar() {
        barLeft.setPosition(forebar_retracted);
        barRight.setPosition(forebar_retracted);
    }
}
