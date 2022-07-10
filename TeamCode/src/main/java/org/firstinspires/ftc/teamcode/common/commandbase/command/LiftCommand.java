package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private LiftSubsystem subsystem;
    private double pos;

    public LiftCommand(LiftSubsystem subsystem, double pos) {
        this.subsystem = subsystem;
        this.pos = pos;
    }

    @Override
    public void execute() {
        subsystem.setPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
