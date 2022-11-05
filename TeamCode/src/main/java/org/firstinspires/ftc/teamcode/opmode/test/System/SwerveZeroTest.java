package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp (name = "SwerveZeroTest2")
public class SwerveZeroTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        waitForStart();
        while (opModeIsActive()) {
            robot.read();

            telemetry.addData("rightfront", robot.drivetrain.modules[0].getModuleRotation());
            telemetry.addData("leftfront", robot.drivetrain.modules[1].getModuleRotation());
            telemetry.addData("Leftrear", robot.drivetrain.modules[2].getModuleRotation());
            telemetry.addData("rightrear", robot.drivetrain.modules[3].getModuleRotation());
            telemetry.update();

//            robot.write();

            PhotonCore.CONTROL_HUB.clearBulkCache();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }
}
