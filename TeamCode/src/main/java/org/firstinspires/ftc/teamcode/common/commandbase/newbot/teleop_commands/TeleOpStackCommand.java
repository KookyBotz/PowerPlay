package org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class TeleOpStackCommand extends SequentialCommandGroup {
    public TeleOpStackCommand(IntakeSubsystem intake) {
        super(
                new InstantCommand(() -> intake.changeStackHeight(0)),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                new InstantCommand(() -> intake.stackHeight = MathUtils.clamp(intake.stackHeight - 1, 0, 4))
        );
    }
}
