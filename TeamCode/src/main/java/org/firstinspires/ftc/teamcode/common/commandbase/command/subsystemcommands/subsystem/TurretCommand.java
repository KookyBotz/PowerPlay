package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class TurretCommand extends InstantCommand {
    public TurretCommand(Robot robot, IntakeSubsystem.TurretState state) {
        super(
                () -> robot.intake.update(state)
        );
    }
}
