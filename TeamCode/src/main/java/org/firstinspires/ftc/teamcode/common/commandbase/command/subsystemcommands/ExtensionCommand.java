package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.test.System.ExtensionTest;

public class ExtensionCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double x;
    private double y;

    public ExtensionCommand(IntakeSubsystem intake, double x, double y) {
        this.intake = intake;
        this.x = x;
        this.y = y;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

    }
}
