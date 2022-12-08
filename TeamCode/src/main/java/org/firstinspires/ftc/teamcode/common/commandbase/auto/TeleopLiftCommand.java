package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class TeleopLiftCommand extends ParallelCommandGroup {
    public TeleopLiftCommand(Robot robot, int position, LiftSubsystem.STATE state) {
        super(
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),
                new InstantCommand(() -> robot.lift.update((state.equals(LiftSubsystem.STATE.EXTEND) ? LiftSubsystem.LatchState.LATCHED : LiftSubsystem.LatchState.UNLATCHED))),
                new LiftPositionCommand(robot.lift, position, 3000, 7500, state.equals(LiftSubsystem.STATE.EXTEND) ? 30 : 10, state.equals(LiftSubsystem.STATE.EXTEND) ? 3000 : 2000, state)
        );
    }
}
