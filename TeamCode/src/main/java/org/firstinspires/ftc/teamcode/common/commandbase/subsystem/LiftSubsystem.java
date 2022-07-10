package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class LiftSubsystem extends SubsystemBase {
    private MotorEx liftA, liftB;
    private PIDController controller;
    private double target, position;

    public LiftSubsystem(MotorEx liftA, MotorEx liftB) {
        this.liftA = liftA;
        this.liftB = liftB;

        this.controller = new PIDController(0, 0, 0);
    }

    public void setPos(double targetPos) {
        target = targetPos;
    }

    @Override
    public void periodic() {
        position = liftA.getCurrentPosition();
        double power = controller.calculate(position, target);
        liftA.set(power);
        liftB.set(power);
    }

    public double getPos() {
        return position;
    }
}
