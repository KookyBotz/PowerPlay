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

// TODO: Test this, I just copied TeleopCycleCommand lmao
public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(Robot robot, GrabPosition state) {
        super(
                // in parallel
                new ParallelCommandGroup(
                        // extend intake slides, grab, and transfer
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarPos)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                                new IntakePositionCommand(robot.intake, state.intPos, 6000, 4500, 20, 3000, IntakeSubsystem.STATE.FAILED_EXTEND),

                                new GrabStackCommand(robot, state),
                                new WaitCommand(200),

                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)),
                                new IntakePositionCommand(robot.intake, 0, 6000, 4500, 10, 3000, IntakeSubsystem.STATE.FAILED_RETRACT)
                                        .alongWith(new WaitCommand(500).andThen(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)))),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new WaitCommand(100)

                        ),

                        // and deposit previous cone
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                new LiftPositionCommand(robot.lift, 610, 6000, 7500, 30, 3000, LiftSubsystem.STATE.FAILED_EXTEND),
                                new WaitCommand(0),
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                new LiftPositionCommand(robot.lift, 15, 6000, 7500, 10, 2000, LiftSubsystem.STATE.FAILED_RETRACT)
                        )
                )
        );
    }
}
