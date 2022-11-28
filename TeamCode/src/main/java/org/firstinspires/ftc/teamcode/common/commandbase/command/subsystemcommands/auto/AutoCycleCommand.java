package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

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
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.KinematicState;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(Robot robot, int pos, double amog) {

    }

    public AutoCycleCommand(Robot robot, KinematicState state) {
        super(
                // TODO replace -0.2 with a target parameter
//                new InstantCommand(() -> robot.intake.newProfile(state.intakeStartingPos + ((0.8 - robot.localizer.getPos().x) / (Math.sin(robot.localizer.getPos().heading)) * 23.5), 1500, 1500)),
                new InstantCommand(() -> robot.intake.newProfile(state.intakeStartingPos, 1500, 1500)),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarStartPos)),
//                new FourbarCommand(robot, IntakeSubsystem.FourbarState.INTAKE),
                new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),

                new WaitCommand(250),
                new LatchCommand(robot, LiftSubsystem.LatchState.LATCHED),

                new LiftCommand(robot, LiftSubsystem.LiftState.HIGH),

                //wait until ready to intake
                new WaitUntilCommand(() -> robot.lift.getPos() > 580),
                new WaitCommand(750),

                new LatchCommand(robot, LiftSubsystem.LatchState.UNLATCHED),
                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
//                new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
                //intake
                new WaitUntilCommand(() -> robot.intake.getPos() > state.intakeStartingPos - 10),
                new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
                new WaitCommand(200),
                // KINEMATICS COMMAND
                new KinematicCommand(robot, state),
                new WaitUntilCommand(() -> robot.intake.getPos() > state.intakeEndPos - 10),
                // END KINEMATICS COMMAND
                new WaitCommand(350),
                new InstantCommand(() -> robot.intake.newProfile(15, 1500, 4000)),
                new WaitCommand(100),
                new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),
                //transfer
                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
                new WaitUntilCommand(() -> robot.intake.getPos() < 20),
                new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT),
                new WaitCommand(500),
                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN)
//                new InstantCommand(() -> robot.intake.newProfile(distance, 600, 1500)),
//                new InstantCommand(() -> robot.intake.resetTimer()),
//                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
//                new InstantCommand(() -> robot.intake.setFourbar(fourbarPos)),
//                new TurretCommand(robot, IntakeSubsystem.TurretState.INTAKE),
//
//                new LiftCommand(robot, LiftSubsystem.LiftState.HIGH),
////                new InstantCommand(() -> robot.lift.newProfile(615, 800, 2500)),
//
//                new WaitUntilCommand(() -> robot.intake.getPos() > distance - 30 && robot.lift.getPos() > 575),
//                new WaitCommand(250),
//
////                new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)),
//                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
//                //intake
//                new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED),
//                new WaitCommand(250),
//                new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
//                new WaitCommand(400),
//                new TurretCommand(robot, IntakeSubsystem.TurretState.DEPOSIT),
//
//                new InstantCommand(() -> robot.intake.newProfile(-5, 1000, 4000)),
//
//                new WaitUntilCommand(() -> robot.lift.getPos() < 10),
//                new WaitUntilCommand(() -> robot.intake.getPos() < 10),
//                new FourbarCommand(robot, IntakeSubsystem.FourbarState.DEPOSIT),
//                new WaitCommand(250),
//                new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN),
//                new WaitCommand(250),
//                new FourbarCommand(robot, IntakeSubsystem.FourbarState.TRANSITION),
//                new WaitCommand(400)
        );
    }
}
