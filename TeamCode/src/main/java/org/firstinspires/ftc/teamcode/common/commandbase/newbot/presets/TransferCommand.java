package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakeSubsystem intake, LiftSubsystem lift) {
        super(
                new LatchCommand(lift, LiftSubsystem.LatchState.UNLATCHED),
                new ConditionalCommand(
                        new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME),
                        () -> intake.clawState.equals(IntakeSubsystem.ClawState.OPEN)
                ),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INWARDS),
                new PivotCommand(intake, IntakeSubsystem.PivotState.PRE_TRANSFER),
                new ConditionalCommand(
                        new WaitCommand(150),
                        new WaitCommand(0),
                        () -> intake.turretState == IntakeSubsystem.TurretState.INTERMEDIATE
                ),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
                new InstantCommand(() -> intake.setTargetPosition(-5)),
                new WaitUntilCommand(() -> intake.isWithinTolerance() && Math.abs(intake.fourbarMotionState.v) <= 0.3),

                new FourbarCommand(intake, IntakeSubsystem.FourbarState.TRANSFER),
                new PivotCommand(intake, IntakeSubsystem.PivotState.TRANSFER),
                new WaitUntilCommand(() -> intake.fourbarMotionState.v == 0),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.INTERMEDIATE),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                new WaitCommand(50),
                new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new InstantCommand(intake::retractReset)
        );
    }
}
