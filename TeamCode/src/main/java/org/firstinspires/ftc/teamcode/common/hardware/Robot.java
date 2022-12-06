package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class Robot {

    public final BNO055IMU imu;

    public SwerveDrivetrain drivetrain;
    public Localizer localizer;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    public Motor.Encoder horizontalEncoder, lateralEncoder;

    private boolean isAuto = false;

    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        this.isAuto = isAuto;
        drivetrain = new SwerveDrivetrain(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        horizontalEncoder = new MotorEx(hardwareMap, "rightFrontMotor").encoder;
        lateralEncoder = new MotorEx(hardwareMap, "leftFrontMotor").encoder;

        intake = new IntakeSubsystem(hardwareMap, isAuto);

        lift = new LiftSubsystem(hardwareMap, isAuto);
    }

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public double getAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }

    public void reset() {
        lift.liftEncoder.resetEncoder();
        intake.extensionEncoder.resetEncoder();
    }

    public void read() {
        intake.read();
        lift.read();
        for (SwerveModule module : drivetrain.modules) {
            module.read();
        }
    }

    public void write() {
        intake.write();
        lift.write();
        if (this.isAuto) {
            drivetrain.writeAuto();
        } else {
            drivetrain.write();
        }
    }
}
