package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LiftExtendCommand extends SequentialCommandGroup {
    public LiftExtendCommand(Robot robot) {
        robot.lift.setPos(robot.lift.high_pos);
    }
}
