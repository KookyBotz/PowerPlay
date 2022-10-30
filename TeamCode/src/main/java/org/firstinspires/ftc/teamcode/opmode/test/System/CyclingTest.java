package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.CycleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.MotionConstraints;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.profiling.AsymmetricMotionProfile;

@Config
@TeleOp(name = "CyclingTest")
public class CyclingTest extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;
    private boolean fA = false;
    private boolean fB = false;
    private boolean fX = false;
    private boolean fY = false;
    private boolean fRB = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setFourbar(0.6);
        robot.reset();
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        // use fallimg edge dedteier
        boolean a = gamepad1.a;
        if (a && !fA) {
            schedule(new InstantCommand(() -> robot.lift.resetTimer())
                    .alongWith(new InstantCommand(() -> robot.lift.setDVA(500, 1500, 7500))));
        }
        boolean fA = a;

        boolean b = gamepad1.b;
        if (b && !fB) {
            schedule(new InstantCommand(() -> robot.lift.setDVA(-500, -1500, -7500))
                    .alongWith(new InstantCommand(() -> robot.lift.resetTimer())));
        }
        fB = b;

        // 600 1500 7500
        boolean x = gamepad1.x;
        if (x && !fX) {
            schedule(
                    new InstantCommand(() -> robot.intake.setMotionProfile(
                    new AsymmetricMotionProfile(robot.intake.getPos(), 400,
                    new MotionConstraints(750, 2500, 2500)))));
        }
        boolean fX = x;

        boolean y = gamepad1.y;
        if (y && !fY) {
            schedule(
                    new InstantCommand(() -> robot.intake.setMotionProfile(
                            // TODO replace 400 with robot.intake.getPos() after testing is complete
                    new AsymmetricMotionProfile(400, 0,
                    new MotionConstraints(-750, -2500, 2500)))));
        }
        fY = y;

        boolean rb = gamepad1.right_bumper;
        if (rb && !fRB) {
            schedule(new SequentialCommandGroup(
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new CycleCommand(robot),
                    new CycleCommand(robot)
            ));
        }
        fRB = rb;

        robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();

        telemetry.addData("liftPos:", robot.lift.getPos());
        telemetry.addData("liftPow:", robot.lift.power);
        telemetry.addData("intakePos:", robot.intake.getPos());
        telemetry.addData("intakePow:", robot.intake.power);
        telemetry.addData("intakeTar:", robot.intake.curState.x);
        telemetry.addData("intakeVel:", robot.intake.curState.v);
        telemetry.addData("intakeAcc:", robot.intake.curState.a / 100);
        telemetry.addData("dt1", robot.intake.profile.calculate(robot.intake.profile.dt1).x);

        double loop = System.currentTimeMillis();
        telemetry.addData("hz ", 1000 / (loop - loopTime));
        telemetry.update();

        loopTime = loop;
        PhotonCore.EXPANSION_HUB.clearBulkCache();
        PhotonCore.CONTROL_HUB.clearBulkCache();

        if (gamepad1.left_bumper) {
            robot.intake.extension.resetEncoder();
            robot.lift.lift.resetEncoder();
        }
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        robot.intake.extension.resetEncoder();
        robot.intake.extension.set(0);
        robot.lift.lift.resetEncoder();
        robot.lift.lift.set(0);
    }
}
