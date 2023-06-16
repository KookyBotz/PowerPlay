package org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.DetectionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarProfiledCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class GroundScoreCommand extends SequentialCommandGroup {
    public GroundScoreCommand(IntakeSubsystem intake) {
        super(
                new DetectionCommand(intake),
                new InstantCommand(() -> intake.setTargetPosition(0)),
                new FourbarProfiledCommand(intake, IntakeSubsystem.FourbarState.GROUND),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT)
        );
    }
}
