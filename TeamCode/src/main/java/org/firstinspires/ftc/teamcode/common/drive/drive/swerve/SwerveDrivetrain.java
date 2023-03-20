package org.firstinspires.ftc.teamcode.common.drive.drive.swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.drive.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.MathUtils;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.*;

@Config
public class SwerveDrivetrain implements Drivetrain {

    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;

    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;
    public static double frontLeftOffset = 2.65, frontRightOffset = 3.64, backLeftOffset = 1.91, backRightOffset = 1.95;

    public static double K_STATIC = 0.03;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public static double minPow = 0.07;
    public static double imuOffset = 0.0;

    public SwerveDrivetrain() {
        frontLeftModule = new SwerveModule(robot.frontLeftMotor, robot.frontLeftServo, new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).zero(frontLeftOffset));
        backLeftModule = new SwerveModule(robot.backLeftMotor, robot.backLeftServo, new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).zero(backLeftOffset));
        backRightModule = new SwerveModule(robot.backRightMotor, robot.backRightServo, new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).zero(backRightOffset));
        frontRightModule = new SwerveModule(robot.frontRightMotor, robot.frontRightServo, new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).zero(frontRightOffset));

        modules = new SwerveModule[]{frontLeftModule, backLeftModule, backRightModule, frontRightModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    public void read() {
        for (SwerveModule module : modules) module.read();
    }

    public void setIMUOffset(double offset) {
        this.imuOffset = offset;
    }

    @Override
    public void set(Pose pose) {
        set(pose, -1);
    }

    @Override
    public void set(Pose pose, double maxPower) {
        double x = pose.x, y = pose.y, head = pose.heading;

        if (maxPower != -1) {
            double r = Math.hypot(x, y);
            x = x / r * maxPower;
            y = y / r * maxPower;
        }

        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);

        ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
        wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};

        max = MathUtils.max(ws);
        //todo integrate motor flipping here
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]) + ((AUTO) ? minPow * Math.signum(ws[i]) : 0));
            m.setTargetRotation(MathUtils.norm(wa[i]));
        }
    }
//
//    public void writeAuto() {
//        for (int i = 0; i < 4; i++) {
//            SwerveModule m = modules[i];
//            if (Math.abs(max) > 1) ws[i] /= max;
//            m.setMotorPower(Math.abs(ws[i] + minPow * Math.signum(ws[i])));
//            m.setTargetRotation(MathUtils.norm(wa[i]));
//        }
//    }

//    public void write() {
//        for (int i = 0; i < 4; i++) {
//            SwerveModule m = modules[i];
//
//        }
//    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
        SwerveModule.K_STATIC = K_STATIC;
    }

    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }
}