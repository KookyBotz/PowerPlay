package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class TeleOpAutoGrabCommand extends SequentialCommandGroup {
    public TeleOpAutoGrabCommand(IntakeSubsystem intake) {
        if (intake.fourbarState != IntakeSubsystem.FourbarState.INTAKE) {
            addCommands(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new FourbarCommand(intake, IntakeSubsystem.FourbarState.INTAKE),
                                    new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                                    new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                                    new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN)
                            ),
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(intake::hasCone),
                                    new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED),
                                    new InstantCommand(() -> intake.setTargetPosition(0)),
                                    new WaitCommand(25),
                                    new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INTERMEDIATE)),
                                    new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                    new WaitUntilCommand(() -> intake.getTargetPosition() <= Globals.INTAKE_EXTENDED_TOLERANCE)
                            )
                    )
            );
        } else {
            if (intake.getPos() >= Globals.INTAKE_ERROR_TOLERANCE) {
                addCommands(
                        new InstantCommand(() -> intake.setTargetPosition(0)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INTERMEDIATE)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                        new WaitUntilCommand(() -> intake.getTargetPosition() <= Globals.INTAKE_EXTENDED_TOLERANCE)
                );
            } else {
                addCommands(new InstantCommand(() -> intake.setTargetPosition(560)));
            }
        }
    }
}


