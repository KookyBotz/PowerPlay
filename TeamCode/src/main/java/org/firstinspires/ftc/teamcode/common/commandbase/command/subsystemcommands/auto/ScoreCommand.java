package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class ScoreCommand extends CommandBase {
    private LiftSubsystem lift;
    private int pos;
    private int vel;
    private int acc;

    public ScoreCommand(LiftSubsystem lift, int pos, int vel, int acc) {
        this.lift = lift;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }

    @Override
    public void execute() {
        lift.setDVA(pos, vel, acc);
        lift.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
