package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class CloseClawCommand extends SequentialCommandGroup {
    public CloseClawCommand(Robot robot) {
        super (
            new InstantCommand(() -> robot.intake.closeClaw())
        );
    }
}
