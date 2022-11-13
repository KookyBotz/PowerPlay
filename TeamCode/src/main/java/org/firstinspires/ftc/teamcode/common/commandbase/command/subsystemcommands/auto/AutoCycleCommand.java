package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(Robot robot, int distance, double fourbarPos) {
        super(
                new InstantCommand(() -> robot.intake.newProfile(distance, 600, 1500)),
                new InstantCommand(() -> robot.intake.resetTimer()),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new InstantCommand(() -> robot.intake.setFourbar(fourbarPos)),
                new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),

                new InstantCommand(() -> robot.lift.newProfile(615, 800, 2500)),

                new WaitUntilCommand(() -> robot.intake.getPos() > distance - 30 && robot.lift.getPos() > 575),
                new WaitCommand(250),

                new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
                //intake
                new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
                new WaitCommand(250),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
                new WaitCommand(400),
                new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),

                new InstantCommand(() -> robot.intake.newProfile(-5, 1000, 4000)),

                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                new WaitUntilCommand(() -> robot.intake.getPos() < 10),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT),
                new WaitCommand(250),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new WaitCommand(250),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
                new WaitCommand(400)
        );
    }
}
