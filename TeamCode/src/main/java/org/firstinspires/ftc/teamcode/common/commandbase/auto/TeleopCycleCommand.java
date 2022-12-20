package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.Consumer;

public class
TeleopCycleCommand extends SequentialCommandGroup {
    public TeleopCycleCommand(Robot robot, Consumer<Boolean> busy) {
        super(
                // in parallel
                new ParallelCommandGroup(
                        // extend intake slides, grab, and transfer
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.INTAKE)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE)),
                                new IntakePositionCommand(robot.intake, 270, 3000, 2000, 10, 2000, IntakeSubsystem.STATE.FAILED_EXTEND),

                                // wait for stuff to stabilize
                                new WaitCommand(600),

                                // grab
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                                new WaitCommand(200),

                                // transfer position
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)),
                                new IntakePositionCommand(robot.intake, 15, 3000, 3000, 10, 3000, IntakeSubsystem.STATE.FAILED_RETRACT),

                                // transfer
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION))

                        ),

                        // and deposit previous cone
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                new LiftPositionCommand(robot.lift, 610, 3000, 7500, 30, 3000, LiftSubsystem.STATE.FAILED_EXTEND),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                                new LiftPositionCommand(robot.lift, 0, 3000, 7500, 10, 2000, LiftSubsystem.STATE.FAILED_RETRACT)
                        )
                ),

                new InstantCommand(() -> busy.accept(false))
        );
    }
}
