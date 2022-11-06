package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp(name = "IntakeEncoderTest")
@Config
public class IntakeEncoderTest extends LinearOpMode {
    public static double fourbarpos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        while (!isStarted()) {
            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.read();
            robot.intake.setFourbar(fourbarpos);
            telemetry.addData("intakepos", robot.intake.getPos());
            telemetry.addData("fourbarpos", robot.intake.getFourbarPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }
}
