package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.geometry.KinematicState;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

// TODO: Test this, I just copied TeleopCycleCommand lmao
public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(Robot robot, KinematicState state) {
        super(
                // in parallel
                new ParallelCommandGroup(
                        // extend intake slides, grab, and transfer
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarStartPos)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE)),
                                new IntakePositionCommand(robot.intake, state.intakeStartingPos, 750, 1500, 10, 3000, IntakeSubsystem.STATE.EXTEND),

                                // wait for stuff to stabilize
                                new WaitCommand(50),

                                // grab
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                                new WaitCommand(200),

                                //move up
                                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarEndPos)),
                                new WaitCommand(100),
                                new IntakePositionCommand(robot.intake, state.intakeEndPos, 750, 1500, 10, 2000, IntakeSubsystem.STATE.EXTEND),

                                // transfer position
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)),
                                new IntakePositionCommand(robot.intake, 15, 1500, 4000, 10, 3000, IntakeSubsystem.STATE.RETRACT),

                                // transfer
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))

                        ),

                        // and deposit previous cone
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                new LiftPositionCommand(robot.lift, 610, 3000, 7500, 30, 3000, LiftSubsystem.STATE.EXTEND),
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                new LiftPositionCommand(robot.lift, 0, 3000, 7500, 10, 2000, LiftSubsystem.STATE.RETRACT)
                        )
                )
        );
    }
}
