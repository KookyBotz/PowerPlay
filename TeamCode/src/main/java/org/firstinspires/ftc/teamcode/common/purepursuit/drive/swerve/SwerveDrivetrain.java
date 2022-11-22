package org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MathUtils;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.Locale;

@Config
public class SwerveDrivetrain implements Drivetrain {

    public SwerveModule leftFrontModule, leftRearModule, rightRearModule, rightFrontModule;
    public SwerveModule[] modules;

    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;
    public static double frontLeftOffset = 1.05 - 0.76, frontRightOffset = 0.07 + 0.76, rearLeftOffset = -2.62 + Math.PI, rearRightOffset = 1.44 + Math.PI;

    public static double frontLeftK = 0.03, frontRightK = 0.05, rearLeftK = 0.04, rearRightK = 0.04;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public SwerveDrivetrain(HardwareMap hardwareMap) {
        leftFrontModule = new SwerveModule(hardwareMap.get(DcMotorEx.class, "leftFrontMotor"), hardwareMap.get(CRServo.class, "leftFrontServo"), new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "leftFrontEncoder"), 2.3).zero(frontLeftOffset + 0.76), frontLeftOffset);
        leftRearModule = new SwerveModule(hardwareMap.get(DcMotorEx.class, "leftRearMotor"), hardwareMap.get(CRServo.class, "leftRearServo"), new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "leftRearEncoder"), 3.3).zero(rearLeftOffset), rearLeftOffset);
        rightRearModule = new SwerveModule(hardwareMap.get(DcMotorEx.class, "rightRearMotor"), hardwareMap.get(CRServo.class, "rightRearServo"), new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "rightRearEncoder"), 3.3).zero(rearRightOffset), rearRightOffset);
        rightFrontModule = new SwerveModule(hardwareMap.get(DcMotorEx.class, "rightFrontMotor"), hardwareMap.get(CRServo.class, "rightFrontServo"), new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "rightFrontEncoder"), 2.3).zero(frontRightOffset - 0.76 + 0.12), frontRightOffset);

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftRearModule, rightRearModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    @Override
    public void set(Pose pose) {
        set(pose, -1);
    }

    @Override
    public void set(Pose pose, double maxPower) {
        System.out.println(pose.toString());
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
        System.out.println(max);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(MathUtils.norm(wa[i]));
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
        leftFrontModule.K_STATIC = frontLeftK;
        rightFrontModule.K_STATIC = frontRightK;
        leftRearModule.K_STATIC = rearLeftK;
        rightRearModule.K_STATIC = rearRightK;
    }

    public String getTelemetry() {
        return leftFrontModule.getTelemetry("leftFrontModule") + "\n" +
                leftRearModule.getTelemetry("leftRearModule") + "\n" +
                rightFrontModule.getTelemetry("rightFrontModule") + "\n" +
                rightRearModule.getTelemetry("rightRearModule") + "\n";
    }
}
