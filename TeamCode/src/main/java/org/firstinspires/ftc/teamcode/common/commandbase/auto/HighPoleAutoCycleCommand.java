package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.FourbarProfiledCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.LiftProfiledCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class HighPoleAutoCycleCommand extends ParallelCommandGroup {
    private SwerveDrivetrain drivetrain;

    public HighPoleAutoCycleCommand(SwerveDrivetrain drivetrain, LiftSubsystem lift, IntakeSubsystem intake, GrabPosition grabPosition, LiftSubsystem.LiftState liftState) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setFourbar(grabPosition.fourbarPos)),
                        new InstantCommand(() -> intake.setTargetPosition(grabPosition.intakeTargetPosition)),
                        new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN_AUTO),
                        new WaitUntilCommand(() -> lift.liftState == LiftSubsystem.LiftState.RETRACTED && intake.isWithinTolerance()),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME),
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
                        new WaitCommand(50),
                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS)
                ),
                new SequentialCommandGroup(
                        new LiftProfiledCommand(lift, liftState),
                        new WaitCommand(75),
                        new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                        new WaitUntilCommand(lift::isWithinTolerance),
                        new WaitCommand(100),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                        new WaitCommand(75),
                        new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
                )
        );

        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.frontLeftModule.setTargetRotation(-PI / 4);
        drivetrain.frontRightModule.setTargetRotation(PI / 4);
        drivetrain.backRightModule.setTargetRotation(-PI / 4);
        drivetrain.backLeftModule.setTargetRotation(PI / 4);
        drivetrain.updateModules();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
