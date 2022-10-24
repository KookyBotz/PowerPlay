package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class FourbarCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double pos;

    public FourbarCommand(IntakeSubsystem intake, double pos) {
        this.intake = intake;
        this.pos = pos;
    }

    @Override
    public void execute() { intake.setFourbar(pos);}
}
