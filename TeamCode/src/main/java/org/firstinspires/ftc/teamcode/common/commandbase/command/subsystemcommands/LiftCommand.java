package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class LiftCommand extends CommandBase {
    private LiftSubsystem lift;
    private int pos;

    public LiftCommand(LiftSubsystem lift, int pos) {
        this.lift = lift;
        this.pos = pos;
    }

    @Override
    public void execute() {
        lift.setPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
