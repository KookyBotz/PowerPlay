package org.firstinspires.ftc.teamcode.common.purepursuit.drive;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MathUtils;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.Arrays;
import java.util.List;

@Config
public class SwerveDrivetrain implements Drivetrain {

    public SwerveModule leftFrontModule, leftRearModule, rightRearModule, rightFrontModule;
    public SwerveModule[] modules;

    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;

    public SwerveDrivetrain(HardwareMap hardwareMap) {
        leftFrontModule = new SwerveModule(hardwareMap, "leftFrontMotor", "leftFrontServo", "leftFrontEncoder");
        leftRearModule = new SwerveModule(hardwareMap, "leftRearMotor", "leftRearServo", "leftRearEncoder");
        rightRearModule = new SwerveModule(hardwareMap, "rightRearMotor", "rightRearServo", "rightRearEncoder");
        rightFrontModule = new SwerveModule(hardwareMap, "rightFrontMotor", "rightFrontServo", "rightFrontEncoder");

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftRearModule, rightRearModule};
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    @Override
    public void set(Pose pose) {
        double x = pose.x, y = pose.y, head = pose.heading;
        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);

        double[] ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
        double[] wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};

        double max = MathUtils.max(ws);
        //todo integrate motor flipping here

        for(int i = 0; i < 4; i++){
            SwerveModule m = modules[i];
            if(max > 1) ws[i] /= max;
            m.setMotorPower(ws[i]);
            m.setTargetRotation(MathUtils.norm(wa[i]));

        }
    }
}
