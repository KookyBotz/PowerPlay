package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class RetractIntakeCommand extends SequentialCommandGroup {
    public RetractIntakeCommand(IntakeSubsystem intake){
        super(
                new ClawCommand(intake, IntakeSubsystem.ClawState.OPEN_AUTO),
                new InstantCommand(()->intake.setTargetPosition(0)),
                new InstantCommand(()->intake.setFourbar(Globals.INTAKE_FOURBAR_INTERMEDIATE))
        );
    }
}
