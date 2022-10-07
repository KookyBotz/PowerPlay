package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class OpMode extends CommandOpMode {
    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
    }
}
