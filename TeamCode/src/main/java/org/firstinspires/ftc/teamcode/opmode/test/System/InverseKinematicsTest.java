package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Kinematics;
import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@Config
@TeleOp (name = "InverseKinematicsTest")
public class InverseKinematicsTest extends OpMode {
    private Robot robot;

    private boolean fB = false;
    private boolean fA = false;

    public static double x, y, l;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {

        boolean a = gamepad1.a;
        if (a && !fA) {
            schedule(new InstantCommand(() -> robot.intake.resetTimer())
                    .alongWith(new InstantCommand(() -> robot.intake.setDVA(400, 750, 2500))));
        }
        boolean fA = a;

        boolean b = gamepad1.b;
        if (b && !fB) {
            schedule(new InstantCommand(() -> robot.intake.setDVA(-400, -750, -2500))
                    .alongWith(new InstantCommand(() -> robot.intake.resetTimer())));
        }
        fB = b;


        double[] kinVals = Kinematics.fourbar(x, y, l);
        telemetry.addData("CurPos", kinVals[0]);
        telemetry.addData("CurAng", kinVals[1]);
        telemetry.update();
    }
}
