package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class C2DepositRetractMediumCommand extends SequentialCommandGroup {
    public C2DepositRetractMediumCommand(LiftSubsystem lift, IntakeSubsystem intake, long delay) {
        super(
                new WaitCommand(delay),
                new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                new WaitCommand(0),
                new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))

        );
    }
}
