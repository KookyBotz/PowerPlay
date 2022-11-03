package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Robot robot, double targetPos, double velocity, double acceleration) {
        super(
                new InstantCommand(() -> robot.intake.openClaw()),
                new ClearFourbarCommand(robot.intake),
                new WaitUntilCommand(() -> robot.intake.getFourbarPos() <= robot.intake.fourbar_transition),
                new InstantCommand(() -> robot.intake.intakeTurret()),
                new InstantCommand(() -> robot.lift.newProfile(targetPos, velocity, acceleration))
        );
    }
}
