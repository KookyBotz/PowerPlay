package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class LiftCommand extends CommandBase {
    LiftSubsystem lift;
    private int distance;
    private int maxV;
    private int maxA;

    public LiftCommand(LiftSubsystem lift, int distance, int maxV, int maxA) {
        this.lift = lift;
        if (distance == 0) {
            distance = -lift.getPos();
            maxV *= -1;
            maxA *= -1;
        }
    }

    @Override
    public void execute() {
        lift.setDVA(distance, maxV, maxA);
        lift.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
