package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

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
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class AutoTransferCommand extends SequentialCommandGroup {
    public AutoTransferCommand(IntakeSubsystem intake, LiftSubsystem lift, GrabPosition position) {
        super(
                new InstantCommand(() -> Globals.MANUAL_ENABLED = false),
                new InstantCommand(() -> intake.setTargetPosition(0)),
                new PivotCommand(intake, IntakeSubsystem.PivotState.PRE_TRANSFER),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
                new WaitCommand(position.turretDelay),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INWARDS),
                new WaitUntilCommand(() -> intake.getPos() < 75 && Math.abs(intake.fourbarMotionState.v) <= 0.3),

                new FourbarCommand(intake, IntakeSubsystem.FourbarState.TRANSFER),
                new PivotCommand(intake, IntakeSubsystem.PivotState.TRANSFER),
                new WaitUntilCommand(() -> Math.abs(intake.fourbarMotionState.v) <= 0.5),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.INTERMEDIATE),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                new InstantCommand(() -> Globals.MANUAL_ENABLED = true)
        );
    }
}
