package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.freightfrenzy.Alliance;

@Config
public class TurretSubsystem extends SubsystemBase {
    private final Servo left;
    private final Servo right;

    public static double left_middle = 0.483;
    public static double right_middle = 0.498;

    public static double offset = 0.44;

    public TurretSubsystem(Servo left, Servo right) {
        this.left = left;
        this.right = right;
    }

    public void middle() {
        left.setPosition(left_middle);
        right.setPosition(right_middle);
    }

    public void left() {
        left.setPosition(left_middle + offset);
        right.setPosition(right_middle + offset);
    }

    public void right() {
        left.setPosition(left_middle - offset);
        right.setPosition(right_middle - offset);
    }

    public void shared(Alliance alliance) {
        if (alliance == Alliance.RED) {
            right();
        } else {
            left();
        }
    }
}
