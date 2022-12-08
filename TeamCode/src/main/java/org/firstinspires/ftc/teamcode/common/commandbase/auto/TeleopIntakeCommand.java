package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class TeleopIntakeCommand extends ParallelCommandGroup {
    public TeleopIntakeCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.INTAKE)),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE))
                )
        );
    }
}
