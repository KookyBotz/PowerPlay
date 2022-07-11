package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class BucketSubsystem extends SubsystemBase {
    private final Servo dump;
    private final Servo gate;

    public static double in_position = 0.65;
    public static double rest_position = 0.6;
    public static double dump_position = 0.845;

    public static double gate_closed = 0.45;
    public static double gate_open = 1;

    public BucketSubsystem(Servo dump, Servo gate) {
        this.dump = dump;
        this.gate = gate;
    }

    public void in() {
        dump.setPosition(in_position);
    }

    public void rest() {
        dump.setPosition(rest_position);
    }

    public void open() {
        gate.setPosition(gate_open);
    }

    public void close() {
        gate.setPosition(gate_closed);
    }

    public void dump() {
        dump.setPosition(dump);
    }
}
