package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets.AutoTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class HighPoleAutoCycleCommand extends ParallelCommandGroup {
    public HighPoleAutoCycleCommand(LiftSubsystem lift, IntakeSubsystem intake, GrabPosition grabPosition, LiftSubsystem.LiftState liftState) {
        super(
                new SequentialCommandGroup(
                        // Extend Outwards
                        new InstantCommand(() -> intake.setTargetPosition(grabPosition.intakeTargetPosition)),
                        new InstantCommand(() -> intake.setFourbarTargetPosition(grabPosition.fourbarPos)),
                        new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),

                        // Wait until the lift is retracting and the intake is within tolerance
                        // NOTE - IsWithinTolerance, checks for when it has 20 ticks or less of error. Can be adjusted
                        // via the Globals.INTAKE_ERROR_TOLERANCE variable in FTCDash.
                        new WaitUntilCommand(() -> lift.liftState == LiftSubsystem.LiftState.RETRACTED && intake.isWithinTolerance()),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.AUTO),
                        new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME + 50),
//                        new InstantCommand(() -> intake.setFourbar(grabPosition.fourbarPos + 0.0975)),
                        new FourbarCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
//                        new WaitCommand(50),
                        new InstantCommand(() -> intake.setPivot(grabPosition.pivotPos)),
                        new WaitCommand(50),
                        new AutoTransferCommand(intake, lift, grabPosition),
                        new InstantCommand(intake::retractReset)
                ),
                new SequentialCommandGroup(
                        new LiftCommand(lift, liftState),
                        new WaitCommand(75),
                        new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                        new WaitUntilCommand(lift::isWithinTolerance),
                        new WaitCommand(50),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                        new WaitCommand(75),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
                )
        );
    }
}
