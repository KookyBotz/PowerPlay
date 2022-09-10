package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intake;

    public IntakeSubsystem(MotorEx intake) {
        this.intake = intake;
    }

    public void run() {
        intake.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.set(1);
    }

    public void reverse() {
        intake.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.set(-1);
    }

    public void stop() {
        intake.set(0);
        double pos = intake.motorEx.getCurrentPosition();

        int targetPos = (int) (pos - (pos % (145.1 / 2.0 * 12.0 / 16.0)));
        intake.motorEx.setTargetPosition(targetPos);
        intake.motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.motorEx.setPower(-1);
    }
}
