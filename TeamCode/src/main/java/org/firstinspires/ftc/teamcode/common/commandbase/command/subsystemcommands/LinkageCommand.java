package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class LinkageCommand extends CommandBase {
    private ArmSubsystem arm;
    private DoubleSupplier supplier;

    public LinkageCommand(ArmSubsystem arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
    }

    @Override
    public void execute() {
        arm.linkage(supplier);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
