package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class ScoreConeCommand extends SequentialCommandGroup {
    public ScoreConeCommand(Robot robot) {
        super(
                new LiftExtendCommand(robot),
                new WaitUntilCommand(() -> robot.lift.getPos() == robot.lift.high_pos),
                // TODO add some sort of scoring mechanism or sequence here, as so far it
                // is just a passive deposit mechanism
                new WaitCommand(300),
                new LiftRetractCommand(robot)
        );
    }
}
