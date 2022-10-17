package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(Robot robot) {
        super (
                new InstantCommand(() -> robot.lift.setDVA(525, 1000, 7500)),
                new InstantCommand(() -> robot.lift.resetTimer()),

                new WaitCommand(400),

                new InstantCommand(() -> robot.lift.setDVA(-525, -1000, -7500)),
                new InstantCommand(() -> robot.lift.resetTimer())
        );
    }
}
