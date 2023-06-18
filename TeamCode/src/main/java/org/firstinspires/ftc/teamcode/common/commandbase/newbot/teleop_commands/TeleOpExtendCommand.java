package org.firstinspires.ftc.teamcode.common.commandbase.newbot.teleop_commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class TeleOpExtendCommand extends SequentialCommandGroup {
    public TeleOpExtendCommand(LiftSubsystem lift, IntakeSubsystem intake) {
        super(
                new ConditionalCommand(
                        new WaitCommand(0),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.setTargetPosition(560)),
                                new WaitUntilCommand(intake::hasCone),
                                new InstantCommand(() -> intake.setTargetPosition(intake.getPos())),
                                new WaitCommand(5),
                                new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME),
                                new InstantCommand(() -> intake.setFourbarTargetPosition(Globals.INTAKE_FOURBAR_PRE_TRANSFER)),
                                new InstantCommand(() -> intake.setPivot(0.37)),
                                new WaitCommand(50),
                                new InstantCommand(() -> intake.setTargetPosition(0)),
                                new WaitCommand(25),
                                new PivotCommand(intake, IntakeSubsystem.PivotState.PRE_TRANSFER),
                                new TurretCommand(intake, IntakeSubsystem.TurretState.INWARDS),
                                new WaitUntilCommand(intake::isWithinTolerance),
                                new InstantCommand(() -> intake.setFourbar(Globals.INTAKE_FOURBAR_TRANSFER)),
                                new PivotCommand(intake, IntakeSubsystem.PivotState.TRANSFER),
                                new WaitCommand(75),
                                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN),
                                new InstantCommand(() -> intake.setFourbar(Globals.INTAKE_FOURBAR_INTERMEDIATE)),
                                new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT),
                                new WaitCommand(50),
                                new LatchCommand(lift, LiftSubsystem.LatchState.LATCHED),
                                new WaitCommand(50),
                                new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                                new InstantCommand(() -> lift.setReady(true))
                        ),
                        lift::isReady
                )
        );
    }
}
