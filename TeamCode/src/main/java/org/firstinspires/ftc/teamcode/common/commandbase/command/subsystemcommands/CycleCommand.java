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

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(Robot robot) {
        super(

                new InstantCommand(() -> robot.intake.newProfile(270, 600, 1500)),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.INTAKE),
                new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),

                new InstantCommand(() -> robot.lift.newProfile(610, 800, 2500)),

                //wait until ready to intake
                new WaitUntilCommand(() -> robot.intake.getPos() > 260 && robot.lift.getPos() > 580),
                new WaitCommand(500),

                new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
                //intake
                new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
                new WaitCommand(200),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
                new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),

                new InstantCommand(() -> robot.intake.newProfile(-5, 750, 2500)),

                //transfer
                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                new WaitUntilCommand(() -> robot.intake.getPos() < 10),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT),
                new WaitCommand(250),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new WaitCommand(400),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
                new WaitCommand(400)
        );
    }
}
