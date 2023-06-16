package org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.DetectionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarProfiledCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class IntermediateStateCommand extends SequentialCommandGroup {
    public IntermediateStateCommand(IntakeSubsystem intake) {
        super(
                new DetectionCommand(intake),
                new InstantCommand(() -> intake.setTargetPosition(0)),
                new FourbarProfiledCommand(intake, IntakeSubsystem.FourbarState.INTERMEDIATE),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INTERMEDIATE),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT)
        );
    }
}
