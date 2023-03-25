package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.DetectionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class GroundScoreCommand extends SequentialCommandGroup {
    public GroundScoreCommand(IntakeSubsystem intake) {
        super(
                new DetectionCommand(intake),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.GROUND),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT)
        );
    }
}
