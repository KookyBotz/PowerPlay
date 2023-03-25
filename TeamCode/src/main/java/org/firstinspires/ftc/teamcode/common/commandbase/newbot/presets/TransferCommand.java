package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakeSubsystem intake, LiftSubsystem lift) {
        super(
                new ConditionalCommand(
                        new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME),
                        () -> intake.clawState.equals(IntakeSubsystem.ClawState.OPEN)
                ),
                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.PRE_TRANSFER)),
                new WaitCommand(Globals.wait5),
                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.PRE_TRANSFER)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INWARDS)),
                new WaitUntilCommand(() -> intake.fourbarMotionState.v == 0).alongWith(
                        new WaitCommand((intake.fourbarState == IntakeSubsystem.FourbarState.INTAKE) ? 400 : 0)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.TRANSFER)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.TRANSFER)),
                new WaitCommand(Globals.wait2),
                new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(Globals.wait3),
                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                new WaitCommand(Globals.wait4),
                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS)),
                new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.LATCHED))
        );
    }
}
