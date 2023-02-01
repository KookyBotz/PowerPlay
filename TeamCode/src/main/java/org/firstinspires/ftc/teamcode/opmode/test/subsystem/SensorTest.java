package org.firstinspires.ftc.teamcode.opmode.test.subsystem;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx.RANGING_PROFILE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.BetterDistanceSensor;

@Config
@TeleOp(name = "SensorTest")
public class SensorTest extends LinearOpMode {
    boolean currentMode = true;
    boolean PcurPress = false;
    boolean PcurPressUp = false;
    boolean PcurPressDown = false;

    private double loopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BetterDistanceSensor sensor = new BetterDistanceSensor(hardwareMap, "distanceSensor", 100, DistanceUnit.CM, RANGING_PROFILE.HIGH_SPEED);

        while (opModeInInit()) {
            telemetry.addLine("Currently in INIT");
            telemetry.update();
        }

        while (opModeIsActive()) {
            double distance = sensor.request();

            boolean curPress = gamepad1.a;
            if (curPress && !PcurPress) {
                currentMode = !currentMode;
                sensor.setMode(currentMode);
            }
            PcurPress = curPress;

            boolean curPressUp = gamepad1.dpad_up;
            boolean curPressDown = gamepad1.dpad_down;
            if (curPressUp && !PcurPressUp) {
                sensor.setRequestRate(sensor.getRequestRate() + 10);
            } else if (curPressDown && !PcurPressDown) {
                sensor.setRequestRate(sensor.getRequestRate() - 10);
            }
            PcurPressUp = curPressUp;
            PcurPressDown = curPressDown;

            telemetry.addData("SENSOR DATA", distance);
            telemetry.addData("READ MODE", currentMode);
            telemetry.addData("REQUEST RATE", sensor.getRequestRate());
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
}
