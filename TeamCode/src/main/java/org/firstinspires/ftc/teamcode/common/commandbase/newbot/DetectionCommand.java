package org.firstinspires.ftc.teamcode.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class DetectionCommand extends SequentialCommandGroup {
    public DetectionCommand(IntakeSubsystem intake) {
        super(
          new ConditionalCommand(
                  new ClawCommand(intake, IntakeSubsystem.ClawState.CLOSED)
                          .alongWith(new WaitCommand(Globals.INTAKE_CLAW_CLOSE_TIME)),
                  new WaitCommand(0),
                  () -> !(intake.hasCone() && intake.clawState == IntakeSubsystem.ClawState.CLOSED)
          )
        );
    }
}
