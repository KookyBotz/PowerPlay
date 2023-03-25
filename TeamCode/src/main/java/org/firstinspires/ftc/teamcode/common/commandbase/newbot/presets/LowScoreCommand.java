package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class LowScoreCommand extends SequentialCommandGroup {
    public LowScoreCommand(IntakeSubsystem intake) {
        super(
                new PivotCommand(intake, IntakeSubsystem.PivotState.LOW),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.LOW)
        );
    }
}
