package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class GrabStackCommand extends SequentialCommandGroup {
    public GrabStackCommand(Robot robot, GrabPosition position) {
        super(
                new WaitUntilCommand(() -> robot.lift.getTargetPos() < 20 && robot.lift.getPos() < 563),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(position.clawDelay),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),
                new WaitCommand(position.upDelay),
                new InstantCommand(() -> robot.intake.setPivot(position.pivotPos))
        );
    }
}
