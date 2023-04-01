package org.firstinspires.ftc.teamcode.common.commandbase.newbot.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class AutoTransferCommand extends SequentialCommandGroup {
    public AutoTransferCommand(IntakeSubsystem intake, LiftSubsystem lift) {
        super(
                new InstantCommand(() -> intake.setTargetPosition(-10)),
                new LatchCommand(lift, LiftSubsystem.LatchState.UNLATCHED),
                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.PRE_TRANSFER)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.PRE_TRANSFER)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.INWARDS)),
                new WaitCommand(Globals.wait5),
                new WaitUntilCommand(() -> intake.getTargetPosition() <= Globals.INTAKE_EXTENDED_TOLERANCE),
                new WaitUntilCommand(() -> intake.fourbarMotionState.v == 0),
                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.TRANSFER)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.TRANSFER)),
                new WaitCommand(400),
                new InstantCommand(() -> intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(Globals.wait3),
                new InstantCommand(() -> intake.update(IntakeSubsystem.FourbarState.INTERMEDIATE)),
                new InstantCommand(() -> intake.update(IntakeSubsystem.PivotState.FLAT)),
                new WaitCommand(Globals.wait4),
                new InstantCommand(() -> intake.update(IntakeSubsystem.TurretState.OUTWARDS))
        );
    }
}
