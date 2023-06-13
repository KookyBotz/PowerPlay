package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.powerplay.Junction;

import java.util.function.BooleanSupplier;

public class TeleOpAutoDepositCommand extends SequentialCommandGroup {
    public TeleOpAutoDepositCommand(LiftSubsystem lift, IntakeSubsystem intake, Junction junction, BooleanSupplier deposit) {
        if (lift.isReady()) {
            if (junction == Junction.HIGH) {
                addCommands(
                        new LiftCommand(lift, LiftSubsystem.LiftState.HIGH)
                                .alongWith(new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED))
                                .alongWith(new WaitUntilCommand(lift::isWithinTolerance)
                                        .andThen(new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE))),
                        new WaitUntilCommand(deposit),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                        new WaitCommand(75),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED)),
                        new InstantCommand(() -> lift.setReady(false))
                );
            } else if (junction == Junction.MEDIUM) {
                addCommands(
                        new LiftCommand(lift, LiftSubsystem.LiftState.MID)
                                .alongWith(new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED))
                                .alongWith(new WaitUntilCommand(lift::isWithinTolerance)
                                        .andThen(new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE))),
                        new WaitUntilCommand(deposit),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                        new WaitCommand(75),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED)),
                        new InstantCommand(() -> lift.setReady(false))
                );
            }
        } else {
            if (junction == Junction.HIGH || junction == Junction.MEDIUM && intake.getPos() <= Globals.INTAKE_ERROR_TOLERANCE) {
                addCommands(
                        new TransferCommand(intake, lift),
                        new InstantCommand(() -> lift.setReady(true))
                );
            } else if (junction == Junction.LOW) {
                addCommands(
                        new InstantCommand(() -> intake.setTargetPosition(-5)),
                        new PivotCommand(intake, IntakeSubsystem.PivotState.LOW),
                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                        new FourbarCommand(intake, IntakeSubsystem.FourbarState.LOW),
                        new WaitUntilCommand(deposit),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(300),
                        new InstantCommand(() -> intake.setTargetPosition(-5)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS))
                );
            } else {
                addCommands(
                        new InstantCommand(() -> intake.setTargetPosition(-5)),
                        new FourbarCommand(intake, IntakeSubsystem.FourbarState.GROUND),
                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                        new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                        new WaitUntilCommand(deposit),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(300),
                        new InstantCommand(() -> intake.setTargetPosition(-5)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                        new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS))
                );
            }
        }
    }
}
