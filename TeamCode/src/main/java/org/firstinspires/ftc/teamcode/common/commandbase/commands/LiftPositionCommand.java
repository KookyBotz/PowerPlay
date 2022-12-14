package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftPositionCommand extends CommandBase {
    private final double position;
    private final double timeout;
    private final double max_v;
    private final double max_a;
    private final double allowed_error;

    private final LiftSubsystem lift;
    private final LiftSubsystem.STATE errorState;

    private ElapsedTime timer;

    public LiftPositionCommand(LiftSubsystem lift, double position, double v, double a, double allowed_error, double timeout, LiftSubsystem.STATE error) {
        this.position = position;
        this.timeout = timeout;
        this.lift = lift;
        this.max_v = v;
        this.max_a = a;
        this.allowed_error = allowed_error;
        this.errorState = error;
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
            lift.newProfile(position, max_v, max_a);
        }

        // didnt reach extension in the allotted time, so set the subsystem to the error state specified
        if (timer.milliseconds() > timeout) {
            lift.state = errorState;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.getPos() - position) < allowed_error || timer.milliseconds() > timeout;
    }
}
