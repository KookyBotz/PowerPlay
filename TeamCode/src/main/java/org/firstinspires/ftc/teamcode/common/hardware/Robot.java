package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.MotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

public class Robot {

    public final BNO055IMU imu;

    public SwerveDrivetrain drivetrain;
    public Localizer localizer;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    public Motor.Encoder horizontalEncoder, lateralEncoder;

//    private int currentModuleIndex = 1;

    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        drivetrain = new SwerveDrivetrain(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        horizontalEncoder = new MotorEx(hardwareMap, "rightFrontMotor").encoder;
        lateralEncoder = new MotorEx(hardwareMap, "leftFrontMotor").encoder;

//        AnalogInput claw2 = hardwareMap.get(AnalogInput.class, "claw");
//        claw2.getVoltage();
//        claw2.getMaxVoltage();

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
        lift.lift.resetEncoder();
        intake.extension.resetEncoder();
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
//        drivetrain.write(currentModuleIndex - 1);
//        currentModuleIndex %= 4;
//        currentModuleIndex++;
        drivetrain.write();
    }
}
