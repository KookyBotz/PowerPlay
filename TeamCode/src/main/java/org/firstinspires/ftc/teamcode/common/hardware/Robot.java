package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class Robot {

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public final BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;

    public SwerveDrivetrain drivetrain;
    public Localizer localizer;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new SwerveDrivetrain(hardwareMap);

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        MotorEx extension = new MotorEx(hardwareMap, "extension");
        Servo   barLeft   = hardwareMap.get(Servo.class, "forebarLeft"),
                barRight  = hardwareMap.get(Servo.class, "forebarRight"),
                claw      = hardwareMap.get(Servo.class, "claw"),
                turret    = hardwareMap.get(Servo.class, "turret");

        intake = new IntakeSubsystem(extension, barLeft, barRight, claw, turret);
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return imuAngle;
    }
}
