package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftProfiledCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class CancelableDepositCommand extends SequentialCommandGroup {
    private boolean cancelled = false;

    public CancelableDepositCommand(LiftSubsystem lift, LiftSubsystem.LiftState liftState, SixConeAutoCommand command) {
        super(
                new LiftProfiledCommand(lift, liftState),
                new InstantCommand(() -> command.canRetractDeposit = true),
                new WaitCommand(75),
                new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                new WaitUntilCommand(lift::isWithinTolerance),
                new InstantCommand(() -> command.canRetractDeposit = false),
                new WaitCommand(100),
                new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                new WaitCommand(75),
                new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
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
