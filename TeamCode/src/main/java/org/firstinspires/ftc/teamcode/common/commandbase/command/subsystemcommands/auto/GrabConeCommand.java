package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class GrabConeCommand extends SequentialCommandGroup {
    public GrabConeCommand(Robot robot) {
        super(
                new IntakeExtendCommand(robot),
                new WaitCommand(500),
        );
    }
}
