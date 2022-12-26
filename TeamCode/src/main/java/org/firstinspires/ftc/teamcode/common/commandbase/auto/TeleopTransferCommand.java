package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class TeleopTransferCommand extends SequentialCommandGroup {
    public TeleopTransferCommand(Robot robot) {
        super(
                new IntakePositionCommand(robot.intake, 0, 1500, 4000, 10, 3000, IntakeSubsystem.STATE.FAILED_RETRACT),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.DEPOSIT))
        );
    }
}
