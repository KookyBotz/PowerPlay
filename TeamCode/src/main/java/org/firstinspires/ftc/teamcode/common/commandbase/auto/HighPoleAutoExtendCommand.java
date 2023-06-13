package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import static java.lang.Math.PI;

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
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class HighPoleAutoExtendCommand extends ParallelCommandGroup {
    private SwerveDrivetrain drivetrain;
    public HighPoleAutoExtendCommand(SwerveDrivetrain drivetrain, LiftSubsystem lift, IntakeSubsystem intake, GrabPosition grabPosition, LiftSubsystem.LiftState liftState) {
        super(
                new SequentialCommandGroup(
                        // Extend Outwards
                        new PivotCommand(intake, IntakeSubsystem.PivotState.FLAT_AUTO),
                        new TurretCommand(intake, IntakeSubsystem.TurretState.OUTWARDS),

                        new WaitCommand(100),

                        new InstantCommand(() -> intake.setTargetPosition(grabPosition.intakeTargetPosition)),
                        new InstantCommand(() -> intake.setFourbarTargetPosition(grabPosition.fourbarPos)),


                        // Wait until the lift is retracting and the intake is within tolerance
                        // NOTE - IsWithinTolerance, checks for when it has 20 ticks or less of error. Can be adjusted
                        // via the Globals.INTAKE_ERROR_TOLERANCE variable in FTCDash.
                        new WaitUntilCommand(() -> lift.liftState == LiftSubsystem.LiftState.RETRACTED && intake.isWithinTolerance()),
                        new ClawCommand(intake, IntakeSubsystem.ClawState.AUTO),
                        new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME + 150),
//                        new InstantCommand(() -> intake.setFourbar(grabPosition.fourbarPos + 0.0975)),
                        new FourbarCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
//                        new WaitCommand(50),
                        new InstantCommand(() -> intake.setPivot(grabPosition.pivotPos)),
                        new WaitCommand(50),
                        new AutoTransferCommand(intake, lift, grabPosition),
                        new InstantCommand(intake::retractReset)
                )
        );

        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize(){
        super.initialize();
        drivetrain.frontLeftModule.setTargetRotation(-PI / 4);
        drivetrain.frontRightModule.setTargetRotation(PI / 4);
        drivetrain.backRightModule.setTargetRotation(-PI / 4);
        drivetrain.backLeftModule.setTargetRotation(PI / 4);
        drivetrain.updateModules();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }
}