package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class ScoreCommand extends CommandBase {
    private LiftSubsystem lift;
    private int pos;

    public ScoreCommand(LiftSubsystem lift, int pos, ) {

    }

    @Override
    public void execute() {
        lift.setDVA(pos, 1500, 7500)
        lift.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
