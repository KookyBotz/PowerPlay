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
                                new IntakePositionCommand(robot.intake, state.intakeStartingPos, 3000, 3000, 20, 3000, IntakeSubsystem.STATE.FAILED_EXTEND),

                                // wait for stuff to stabilize
                                new WaitCommand(300),

                                // grab
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                                new WaitCommand(600),

                                //move up
                                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarEndPos)),
                                new IntakePositionCommand(robot.intake, state.intakeEndPos, 750, Integer.MAX_VALUE, 10, 2000, IntakeSubsystem.STATE.FAILED_EXTEND),
                                new WaitCommand(250),
                                // transfer position
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),
                                new IntakePositionCommand(robot.intake, 15, 3000, 3000, 10, 3000, IntakeSubsystem.STATE.FAILED_RETRACT)
                                        .alongWith(new WaitCommand(500).andThen(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)))),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),


                                // transfer
                                new WaitCommand(750),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN))

                        ),

                        // and deposit previous cone
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                new LiftPositionCommand(robot.lift, 610, 3000, 7500, 30, 3000, LiftSubsystem.STATE.FAILED_EXTEND),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                new LiftPositionCommand(robot.lift, 0, 3000, 7500, 10, 2000, LiftSubsystem.STATE.FAILED_RETRACT)
                        )
                )
        );
    }
}
