package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class RetractDepositCommand extends SequentialCommandGroup {
    public RetractDepositCommand(LiftSubsystem lift) {
        super(
                new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED),
                new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED, true))
        );
    }
}
