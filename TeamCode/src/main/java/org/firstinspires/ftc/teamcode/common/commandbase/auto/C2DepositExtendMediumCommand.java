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

public class C2DepositExtendMediumCommand extends SequentialCommandGroup {
    public C2DepositExtendMediumCommand(LiftSubsystem lift, IntakeSubsystem intake) {
        super(
                new LiftCommand(lift, LiftSubsystem.LiftState.MID),
                new WaitCommand(75),
                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED),
                new WaitUntilCommand(lift::isWithinTolerance)

        );
    }
}
