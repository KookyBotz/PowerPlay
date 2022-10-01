package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(Robot robot) {
        super(
            new InstantCommand(() -> robot.intake.extensionIn())
        );
    }
}
