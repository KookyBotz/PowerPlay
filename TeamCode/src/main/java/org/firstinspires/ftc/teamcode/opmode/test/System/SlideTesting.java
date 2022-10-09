package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class SlideTesting extends CommandOpMode {

    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void run() {
        if (gamepad1.a) {
            schedule(new IntakeExtendCommand(robot));
        }
    }
}
