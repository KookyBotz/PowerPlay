package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.LinkageCommand;
import org.firstinspires.ftc.teamcode.common.freightfrenzy.Alliance;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class SharedCommand extends SequentialCommandGroup {
    public SharedCommand(Robot robot, Alliance alliance, BooleanSupplier outtake, DoubleSupplier linkage, DoubleSupplier arm, Consumer<Boolean> done) {
        super(
                new InstantCommand(() -> robot.bucket.close()),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitCommand(50),
                new InstantCommand(() -> robot.intake.reverse()),
                new InstantCommand(() -> robot.arm.armShared()),
                new WaitCommand(75),
                new InstantCommand(() -> robot.bucket.rest()),
                new WaitUntilCommand(() -> robot.arm.pos() > 100),
                new InstantCommand(() -> robot.turret.shared(alliance)),
                new InstantCommand(robot.intake::stop),
                new WaitUntilCommand(() -> robot.arm.pos() > 650),
                new InstantCommand(() -> robot.bucket.in()),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(outtake),
                        new PerpetualCommand(
                                new LinkageCommand(robot.arm, linkage)
                        ),
                        new PerpetualCommand(
                                new ArmCommand(robot.arm, arm)
                        )
                ),
                new InstantCommand(() -> robot.bucket.dump()),
                new WaitCommand(50),
                new InstantCommand(() -> robot.bucket.open()),
                new WaitCommand(200),
                new InstantCommand(() -> robot.bucket.rest()),
                new InstantCommand(() -> robot.turret.middle()),
                new InstantCommand(() -> robot.arm.linkageIn()),
                new InstantCommand(() -> robot.arm.armIn()),
                new WaitUntilCommand(() -> robot.arm.pos() < 300),
                new InstantCommand(() -> robot.bucket.in()),
                new WaitUntilCommand(() -> robot.arm.pos() < 50),
                new WaitCommand(150),
                new InstantCommand(() -> robot.intake.run()),
                new InstantCommand(() -> done.accept(true))
        );
    }
}
