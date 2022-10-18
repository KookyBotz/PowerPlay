package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Robot robot, int distance, int maxV, int maxA) {
        if (distance == 0) {
            distance = robot.intake.getPos();
        }
        super(
            new InstantCommand(() -> robot.lift.resetTimer()),
            new InstantCommand(() -> robot.lift.setDVA())
        );
    }
}
