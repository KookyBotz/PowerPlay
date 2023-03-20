package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LowScoreCommand extends SequentialCommandGroup {
    public LowScoreCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new WaitCommand(100)
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION)),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.FLAT)),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE))
        );
    }
}
