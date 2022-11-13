package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(Robot robot) {
        super(
            new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
            new InstantCommand(() -> robot.intake.newProfile(-5, 750, 2500)),
            new WaitUntilCommand(() -> robot.intake.getPos() < 10),
            new WaitUntilCommand(() -> robot.lift.getPos() < 10),
            new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),
            new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT)
        );
    }
}
