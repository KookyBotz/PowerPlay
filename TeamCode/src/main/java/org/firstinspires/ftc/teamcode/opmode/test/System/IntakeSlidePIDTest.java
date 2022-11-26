package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Config
@TeleOp(name = "Intake PID Test")

public class IntakeSlidePIDTest extends CommandOpMode {

    private ElapsedTime timer;
    private Robot robot;

    boolean pDA = false;

    public static int intakePos = 10;
    int newTargetPos = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot.intake.setFourbar(0.6);
        robot.intake.extension.set(-0.4);
//        robot.lift.lift.set(-0.3);
        robot.intake.setFourbar(robot.intake.fourbar_transition);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();

        if (newTargetPos != intakePos) {
            schedule(new InstantCommand(() -> robot.intake.newProfile(newTargetPos, 1500, 4000)));
        }
        newTargetPos = intakePos;

        robot.intake.loop();
        CommandScheduler.getInstance().run();

        robot.write();


        telemetry.addData("intakePos", robot.intake.getPos());
        telemetry.addData("intakeError", intakePos);
        telemetry.update();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
}
