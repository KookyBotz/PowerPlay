package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class CancelableGrabCommand extends SequentialCommandGroup {
    private boolean cancelled = false;

    public CancelableGrabCommand(LiftSubsystem lift, IntakeSubsystem intake, GrabPosition grabPosition, SixConeAutoCommand command) {
        super(
                new InstantCommand(() -> intake.setFourbar(grabPosition.fourbarPos)),
                new InstantCommand(() -> intake.setTargetPosition(grabPosition.intakeTargetPosition)),
                new InstantCommand(() -> command.canRetractIntake = true),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN_AUTO),
                new WaitUntilCommand(() -> lift.liftState == LiftSubsystem.LiftState.RETRACTED && intake.isWithinTolerance()),
                new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED),
                new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME),
                new InstantCommand(() -> command.canRetractIntake = false),
                new InstantCommand(() -> command.stackHeight--),
                new InstantCommand(() -> intake.setFourbarTargetPosition(Globals.INTAKE_FOURBAR_PRE_TRANSFER)),
                new InstantCommand(() -> intake.setPivot(grabPosition.pivotPos)),
                new WaitCommand(50),
                new InstantCommand(() -> intake.setTargetPosition(0)),
                new WaitCommand(25),
                new PivotCommand(intake, IntakeSubsystem.PivotState.PRE_TRANSFER),
                new WaitCommand(grabPosition.turretDelay),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INWARDS),
                new WaitUntilCommand(intake::isWithinTolerance),
                new InstantCommand(() -> intake.setFourbar(Globals.INTAKE_FOURBAR_TRANSFER)),
                new PivotCommand(intake, IntakeSubsystem.PivotState.TRANSFER),
                new WaitCommand(75),
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                new InstantCommand(() -> intake.setFourbar(Globals.INTAKE_FOURBAR_INTERMEDIATE)),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED),
                new WaitCommand(65),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new InstantCommand(() -> command.canDeposit = true),
                new InstantCommand(() -> command.intaking = false)
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || cancelled;
    }

    public void cancel() {
        cancelled = true;
    }
}
