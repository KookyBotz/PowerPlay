package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends OpMode {
    Robot robot;
    private double loopTime = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void loop() {
        robot.read();
        telemetry.addData("liftPos", robot.lift.getPos());
        telemetry.addData("intakePos", robot.intake.getPos());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();

        robot.lift.lift.set(gamepad1.left_stick_y);
        robot.intake.extension.set(gamepad1.right_stick_y);
    }
}
