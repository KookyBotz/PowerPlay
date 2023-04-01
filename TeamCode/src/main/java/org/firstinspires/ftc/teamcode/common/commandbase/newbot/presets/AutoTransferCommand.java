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
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class AutoTransferCommand extends SequentialCommandGroup {
    public AutoTransferCommand(IntakeSubsystem intake, LiftSubsystem lift, GrabPosition position) {
        super(
                new InstantCommand(() -> Globals.MANUAL_ENABLED = false),
                new InstantCommand(() -> intake.setTargetPosition(0)),
                new PivotCommand(intake, IntakeSubsystem.PivotState.PRE_TRANSFER),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
                new WaitCommand(position.upDelay),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INWARDS),
                new WaitUntilCommand(() -> intake.isWithinTolerance() && intake.fourbarMotionState.v == 0),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.TRANSFER),
                new PivotCommand(intake, IntakeSubsystem.PivotState.TRANSFER),
                new WaitUntilCommand(() -> intake.fourbarMotionState.v == 0),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                new WaitCommand(20),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.INTERMEDIATE),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                new WaitCommand(50),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new InstantCommand(() -> Globals.MANUAL_ENABLED = true)
        );
    }
}
