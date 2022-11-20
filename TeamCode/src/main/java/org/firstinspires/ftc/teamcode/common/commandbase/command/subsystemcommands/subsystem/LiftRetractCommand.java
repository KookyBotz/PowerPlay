package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class LiftRetractCommand extends InstantCommand {
    public LiftRetractCommand(Robot robot) {
        super (
                () -> robot.lift.update(LiftSubsystem.LiftState.RETRACTED)
        );
    }
}
