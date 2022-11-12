package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FourbarCommand extends InstantCommand {
    public FourbarCommand(Robot robot, IntakeSubsystem.FourbarState state) {
        super(
                () -> robot.intake.update(state)
        );
    }
}
