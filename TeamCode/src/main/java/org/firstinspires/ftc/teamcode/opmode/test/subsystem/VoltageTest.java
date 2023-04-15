package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class VoltageTest extends OpMode {
    private AnalogInput input;
    double pVoltage;
    double pVoltage_a;

    double pTime;
    @Override
    public void init() {
        input = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        hardwareMap.get(CRServo.class, "frontLeftServo").setPower(0.2);
    }

    @Override
    public void loop() {
        double v = input.getVoltage();
        double v2 = reLinearize(input.getVoltage());

        telemetry.addData("v", v);
        telemetry.addData("v adjusted", v2);

        double time = System.nanoTime();
        double diff = (time - pTime);

        double a = v - pVoltage * 10000000;
        double b = v2 - pVoltage_a * 10000000;

        telemetry.addData("dv", a/diff);
        telemetry.addData("dv2", b/diff);

        pTime = time;
        pVoltage = v;
        pVoltage_a = v2;
    }

    public static double reLinearize(double v){
        double v2 = 3.3-v;
        double alpha = v2 * ((3.3 * v2) - 16.335);
        double beta = (v2 * v2) - (3.3 * v2) - 5.445;
        return 3.3 - (alpha / beta);
    }
}
