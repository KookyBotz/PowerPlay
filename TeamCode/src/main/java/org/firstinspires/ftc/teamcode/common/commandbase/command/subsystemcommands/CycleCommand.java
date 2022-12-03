package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.FourbarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LatchCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(Robot robot) {
        super(
                // deposit/intake movements
                new InstantCommand(() -> robot.intake.newProfile(270, 750, 1500)),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),

                new LatchCommand(robot, LiftSubsystem.LatchState.LATCHED),
                new LiftCommand(robot, LiftSubsystem.LiftState.HIGH),

                new WaitUntilCommand(() -> robot.intake.getPos() > 75),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.INTAKE),
                new WaitUntilCommand(() -> robot.lift.getPos() > 580),
                new WaitCommand(200),

                new LatchCommand(robot, LiftSubsystem.LatchState.UNLATCHED),
                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
                //intake

                new WaitUntilCommand(() -> robot.intake.getPos() > 260),
                new WaitCommand(50),
                new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
                new WaitCommand(200),
                new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT),

                new InstantCommand(() -> robot.intake.newProfile(15, 1500, 4000)),

                //transfer
                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                new WaitUntilCommand(() -> robot.intake.getPos() < 25),
                new WaitCommand(250),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN)
        );
    }
}
