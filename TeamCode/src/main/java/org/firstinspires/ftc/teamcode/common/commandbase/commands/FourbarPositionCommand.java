package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class FourbarPositionCommand extends CommandBase {
    private final double position;
    private final double v;
    private final double a;

    private final IntakeSubsystem intake;

    public FourbarPositionCommand(IntakeSubsystem intake, double position, double v, double a) {
        this.position = position;
        this.v = v;
        this.a = a;

        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.fourbarProfile(position, v, a);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
