package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Robot robot, LiftSubsystem.LiftState state) {
        super(
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new WaitCommand(250),
                new LatchCommand(robot, LiftSubsystem.LatchState.LATCHED),
                new ClearFourbarCommand(robot.intake),
                new WaitUntilCommand(() -> robot.intake.getFourbarPos() <= robot.intake.fourbar_transition),
                new WaitCommand(250),
                new InstantCommand(() -> robot.lift.update(state)),
//                new InstantCommand(() -> robot.lift.newProfile(targetPos, velocity, acceleration)),
                new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE)
        );
    }
}
