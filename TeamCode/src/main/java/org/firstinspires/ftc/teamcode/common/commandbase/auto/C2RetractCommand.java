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

public class C2RetractCommand extends SequentialCommandGroup {
    public C2RetractCommand(IntakeSubsystem intake, GrabPosition grabPosition) {
        super(
                new ClawCommand(intake, IntakeSubsystem.ClawState.AUTO),
                new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME + 50),
                new FourbarCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
                new InstantCommand(() -> intake.setPivot(grabPosition.pivotPos)),
                new WaitCommand(50),
                new AutoTransferCommand(intake, grabPosition),
                new InstantCommand(intake::retractReset)
        );
    }
}
