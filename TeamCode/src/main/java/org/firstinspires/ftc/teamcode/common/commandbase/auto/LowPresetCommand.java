package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LowPresetCommand extends SequentialCommandGroup {
    public LowPresetCommand(final Robot robot) {
        super(
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.CLOSED)),
                new IntakePositionCommand(robot.intake, -5, 6000, 2500, 20, 3000, IntakeSubsystem.STATE.FAILED_RETRACT)
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.PivotState.SCORE)),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.SCORE)),
//                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.TurretState.INTAKE))
        );
    }
}
