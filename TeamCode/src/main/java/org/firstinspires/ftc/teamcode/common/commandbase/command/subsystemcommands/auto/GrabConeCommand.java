package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class GrabConeCommand extends SequentialCommandGroup {
    public GrabConeCommand(Robot robot) {
        super(
                //
                //.alongWith(new ForebarCommand(robot.intake, robot.intake, ))
        );
    }
}
