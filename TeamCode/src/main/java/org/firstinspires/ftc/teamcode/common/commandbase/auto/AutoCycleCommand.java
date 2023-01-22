package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(Robot robot, GrabPosition state) {
        super(
                // in parallel
                new ParallelCommandGroup(
                        // extend intake slides, grab, and transfer
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarPos)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new IntakePositionCommand(robot.intake, state.intPos, 6000, 4500, 20, 3000, IntakeSubsystem.STATE.FAILED_EXTEND)
                                        .alongWith(new WaitCommand(75).andThen(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE)))),

                                new WaitCommand(200),
                                new GrabStackCommand(robot, state),

                                new IntakePositionCommand(robot.intake, -5, 6000, 4500, 20, 3000, IntakeSubsystem.STATE.FAILED_RETRACT)
                                        .alongWith(
                                                new WaitCommand(150)
                                                        .andThen(new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_auto_transfer)))
                                                        .andThen(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)))
                                        ),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                                new WaitCommand(50),
                                new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_flat)),
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new WaitCommand(100)
                        ),

                        // and deposit previous cone
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                new LiftPositionCommand(robot.lift, 585, 6000, 7500, 40, 3000, LiftSubsystem.STATE.FAILED_EXTEND),
                                new WaitCommand(250),
                                new LiftPositionCommand(robot.lift, 0, 6000, 7500, 10, 2000, LiftSubsystem.STATE.FAILED_RETRACT)
                                        .alongWith(new WaitCommand(50).andThen(new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED))))
                        )
                )
        );
    }
}
