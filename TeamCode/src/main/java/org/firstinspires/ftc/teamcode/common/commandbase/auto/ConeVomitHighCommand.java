package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.Consumer;

public class
ConeVomitHighCommand extends SequentialCommandGroup {
    public ConeVomitHighCommand(Robot robot, Consumer<Boolean> busy) {
        super(
                new SequentialCommandGroup(
                        // in parallel
                        new ParallelCommandGroup(
                                // extend intake slides, grab, and transfer
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                        new InstantCommand(() -> robot.intake.setFourbar(IntakeSubsystem.fourbar_extended)),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
                                        new IntakePositionCommand(robot.intake, 270, 4000, 4000, 20, 3000, IntakeSubsystem.STATE.FAILED_EXTEND)
                                                .alongWith(new WaitCommand(75).andThen(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE)))),
//                                        new WaitUntilCommand(() -> robot.intake.getPos() > 479),
                                        new WaitUntilCommand(() -> robot.lift.getTargetPos() < 20 && robot.lift.getPos() < 563),
                                        new IntakePositionCommand(robot.intake, 485, 4000, 4000, 20, 3000, IntakeSubsystem.STATE.FAILED_EXTEND),
                                        new WaitCommand(200),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),

                                        new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_auto_transfer)),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT)),
                                        new IntakePositionCommand(robot.intake, -5, 6000, 4000, 20, 3000, IntakeSubsystem.STATE.FAILED_RETRACT),


                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                                        new InstantCommand(() -> robot.intake.setPivot(IntakeSubsystem.pivot_flat)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                                        new WaitCommand(100)
                                ),

                                // and deposit previous cone
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.LATCHED)),
                                        new LiftPositionCommand(robot.lift, 580, 6000, 7500, 40, 3000, LiftSubsystem.STATE.FAILED_EXTEND),
                                        new WaitCommand(50),
                                        new LiftPositionCommand(robot.lift, 0, 6000, 7500, 10, 2000, LiftSubsystem.STATE.FAILED_RETRACT)
                                                .alongWith(new WaitCommand(50).andThen(new InstantCommand(() -> robot.lift.update(LiftSubsystem.LatchState.UNLATCHED))))
                                )
                        ),
                        new InstantCommand(() -> busy.accept(false))
                )

        );
    }
}
