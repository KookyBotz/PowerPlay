package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Kinematics;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.test.System.ExtensionTest;

public class ExtensionCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double[] points;

    public ExtensionCommand(IntakeSubsystem intake, double x, double y) {
        this.intake = intake;
        this.points = Kinematics.forebar(x, y, intake.FOREBAR_LENGTH);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

    }
}
