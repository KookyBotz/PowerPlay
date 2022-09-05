package org.firstinspires.ftc.teamcode.common.freightfrenzy;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveDrivetrain;

public class SwerveRobot {
    private final Object IMULock = new Object();
    private double imuAngle = 0;
    @GuardedBy("IMULock")
    public final BNO055IMU imu;
    public SwerveDrivetrain drivetrain;
    private Thread imuThread;

    public SwerveRobot(HardwareMap hardwareMap) {

        drivetrain = new SwerveDrivetrain(hardwareMap);

        synchronized (IMULock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (IMULock) {
                    imuAngle = imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

    public double getAngle(){
        return imuAngle;
    }
}
