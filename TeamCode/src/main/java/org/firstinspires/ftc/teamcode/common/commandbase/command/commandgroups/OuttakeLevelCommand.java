package org.firstinspires.ftc.teamcode.common.commandbase.command.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class OuttakeLevelCommand extends SequentialCommandGroup{
    public OuttakeLevelCommand(Robot robot, double level){
        double target = level * 100;
        addCommands(
                new LiftCommand(robot.liftSubsystem, target),
                new WaitUntilCommand(()->robot.liftSubsystem.getPos()>target-25)
        );
    }
}
