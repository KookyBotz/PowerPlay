package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
    private ArmSubsystem arm;
    private DoubleSupplier supplier;

    public ArmCommand(ArmSubsystem arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
    }

    @Override
    public void execute() {
        arm.adjustArm(supplier);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
