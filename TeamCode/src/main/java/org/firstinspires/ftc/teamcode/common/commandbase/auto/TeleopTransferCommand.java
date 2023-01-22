package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class TeleopTransferCommand extends SequentialCommandGroup {
    public TeleopTransferCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                new WaitUntilCommand(() -> robot.lift.getTargetPos() < 20 && robot.lift.getPos() < 563),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
//                new WaitCommand(150), optional, likely not needed here
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),

                new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_auto_transfer)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)),
                new IntakePositionCommand(robot.intake, -5, 6000, 2500, 20, 3000, IntakeSubsystem.STATE.FAILED_RETRACT),

                new WaitCommand(200),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                new WaitCommand(50),
                new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_flat)),
                new WaitCommand(200),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),
                new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_pitch_down))
        );
    }
}
