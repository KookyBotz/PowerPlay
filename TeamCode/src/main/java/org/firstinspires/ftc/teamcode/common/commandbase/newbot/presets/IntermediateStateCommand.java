package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.DetectionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class IntermediateStateCommand extends SequentialCommandGroup {
    public IntermediateStateCommand(IntakeSubsystem intake) {
        super(
                new DetectionCommand(intake),
                new InstantCommand(() -> intake.setTargetPosition(-10)),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.INTERMEDIATE),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INTERMEDIATE),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT)
        );
    }
}
