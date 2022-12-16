package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.geometry.KinematicState;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class KinematicCommand extends SequentialCommandGroup {
    public KinematicCommand(Robot robot, KinematicState state, boolean kinematic) {
        if (kinematic) {
            addCommands(
                    new InstantCommand(() -> robot.intake.setFourbar(state.fourbarEndPos)),
                    new IntakePositionCommand(robot.intake, state.intakeEndPos, 750, Integer.MAX_VALUE, 10, 2000, IntakeSubsystem.STATE.FAILED_EXTEND),
                    new WaitCommand(250)
            );
        } else {
            addCommands(new InstantCommand());
        }
    }
}
