package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.OpMode;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends OpMode {
//    AnalogInput mod1, mod2, mod3, mod4;
    Robot robot;
    private double loopTime = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, true);
//        mod1 = hardwareMap.get(AnalogInput.class, "mod1");
//        mod2 = hardwareMap.get(AnalogInput.class, "mod2");
//        mod3 = hardwareMap.get(AnalogInput.class, "mod3");
//        mod4 = hardwareMap.get(AnalogInput.class, "mod4");
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        robot.read();
//        telemetry.addData("mod1volt", mod1.getVoltage());
//        telemetry.addData("mod2volt", mod2.getVoltage());
//        telemetry.addData("mod3volt", mod3.getVoltage());
//        telemetry.addData("mod4volt", mod4.getVoltage());
        telemetry.addData("liftPos", robot.lift.getPos());
        telemetry.addData("intakePos", robot.intake.getPos());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        PhotonCore.EXPANSION_HUB.clearBulkCache();
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
}
