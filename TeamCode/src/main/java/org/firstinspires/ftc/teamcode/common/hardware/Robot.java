package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.BucketSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class Robot {

    public IntakeSubsystem intake;
    public ArmSubsystem arm;
    public BucketSubsystem bucket;
    public TurretSubsystem turret;

    public Robot(HardwareMap hardwareMap) {

        MotorEx intakeM = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intakeM.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeM.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = new IntakeSubsystem(intakeM);

        Servo dump = hardwareMap.get(Servo.class, "dump");
        Servo gate = hardwareMap.get(Servo.class, "gate");

        bucket = new BucketSubsystem(dump, gate);
    }
}
