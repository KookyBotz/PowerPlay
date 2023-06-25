package org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarProfiledCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.opmode.teleop.Teleop;

import java.util.function.BooleanSupplier;

public class TeleOpAutoGrabCommand extends SequentialCommandGroup {
    public TeleOpAutoGrabCommand(IntakeSubsystem intake, BooleanSupplier override) {
        if (intake.fourbarState != IntakeSubsystem.FourbarState.INTAKE) {
            addCommands(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new FourbarProfiledCommand(intake, IntakeSubsystem.FourbarState.INTAKE),
                                    new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                                    new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                                    new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                                    new InstantCommand(() -> Teleop.autoGrabActive = true)
                            ),
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(()->intake.hasCone() || override.getAsBoolean()),
                                    new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED),
                                                    new WaitCommand(35),
                                                    new InstantCommand(() -> Teleop.autoGrabActive = false),
                                                    new InstantCommand(() -> intake.setTargetPosition(0)),
                                                    new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INTERMEDIATE)),
                                                    new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                                                    new WaitUntilCommand(() -> intake.getTargetPosition() <= Globals.INTAKE_EXTENDED_TOLERANCE)
                                            ),
                                            new WaitCommand(0),
                                            () -> Teleop.autoGrabActive
                                    )
                            )
                    )
            );
        } else {
            if (intake.getPos() >= Globals.INTAKE_ERROR_TOLERANCE) {
                addCommands(
                        new InstantCommand(() -> Teleop.autoGrabActive = false),
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


