package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.purepursuit.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.SwerveDrivetrain;

public class SwerveChassis {
    public final BNO055IMU imu;
    public SwerveDrivetrain drivetrain;

    public SwerveChassis(HardwareMap hardwareMap) {

        drivetrain = new SwerveDrivetrain(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }
}
