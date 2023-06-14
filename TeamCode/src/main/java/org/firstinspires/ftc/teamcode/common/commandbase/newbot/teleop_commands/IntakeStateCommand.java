package org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class IntakeStateCommand extends SequentialCommandGroup {
    public IntakeStateCommand(IntakeSubsystem intake) {
        super(
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.INTAKE),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN)
        );
    }
}
