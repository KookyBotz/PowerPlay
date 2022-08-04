package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MecanumDrivetrain implements Drivetrain {

    private final DcMotorEx fl, fr, bl, br;

    public MecanumDrivetrain(DcMotorEx fl, DcMotorEx bl, DcMotorEx fr, DcMotorEx br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    @Override
    public void set(Pose pose) {
        double frontLeft = pose.x + pose.y + pose.heading;
        double backLeft = pose.x - pose.y + pose.heading;
        double frontRight = pose.x - pose.y - pose.heading;
        double backRight = pose.x + pose.y - pose.heading;

        List<Double> vals = new ArrayList<>();
        vals.add(frontLeft);
        vals.add(backLeft);
        vals.add(frontRight);
        vals.add(backRight);

        double absMax = Math.max(Collections.max(vals), -Collections.min(vals));
        if (absMax > 1) {
            frontLeft /= absMax;
            frontRight /= absMax;
            backLeft /= absMax;
            backRight /= absMax;
        }

        fl.setPower(frontLeft);
        bl.setPower(backLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
    }
}
