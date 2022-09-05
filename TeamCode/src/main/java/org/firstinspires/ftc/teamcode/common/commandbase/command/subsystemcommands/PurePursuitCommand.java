package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.purepursuit.path.PurePursuitPath;

public class PurePursuitCommand extends CommandBase {

    private final PurePursuitPath path;
    private boolean running;

    public PurePursuitCommand(PurePursuitPath path) {
        this.path = path;
    }

    @Override
    public void execute() {
        running = path.update();
    }

    public boolean isFinished() {
        return !running;
    }
}
