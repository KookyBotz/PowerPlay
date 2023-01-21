package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    private Servo a, b, c, d, e, f;

    @Override
    public void init() {
        a = hardwareMap.servo.get("claw");
        b = hardwareMap.servo.get("turret");
        c = hardwareMap.servo.get("fourbarLeft");
        d = hardwareMap.servo.get("fourbarRight");
        e = hardwareMap.servo.get("pivot");
        f = hardwareMap.servo.get("latch");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            a.setPosition(1);
        }

        if(gamepad1.b){
            b.setPosition(1);
        }

        if(gamepad1.x){
            c.setPosition(1);
        }

        if(gamepad1.y){
            d.setPosition(0);
        }

        if(gamepad1.dpad_down){
            e.setPosition(1);
        }

        if(gamepad1.dpad_up){
            f.setPosition(1);
        }
    }
}
