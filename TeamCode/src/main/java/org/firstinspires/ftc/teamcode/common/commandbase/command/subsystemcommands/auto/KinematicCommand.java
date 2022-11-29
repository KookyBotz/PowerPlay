package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.KinematicState;

public class KinematicCommand extends SequentialCommandGroup {
    public KinematicCommand(Robot robot, KinematicState state, boolean noDelay) {
        super(
                new InstantCommand(() -> robot.intake.newProfile(state.intakeEndPos, state.intakeVelo, state.intakeAccel)),
                new WaitCommand(noDelay ? 10 : 250),
                new InstantCommand(() -> robot.intake.setFourbar(state.fourbarEndPos))
        );
    }
}
