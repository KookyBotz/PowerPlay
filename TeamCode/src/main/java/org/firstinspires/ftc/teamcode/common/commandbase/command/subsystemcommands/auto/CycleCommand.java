package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(Robot robot) {
        new ParallelCommandGroup(
            new InstantCommand(() -> robot.intake.setExtension(500)),
            new InstantCommand(() -> robot.intake.resetTimer())
        );
    }
}
