package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Autonomous(name = "Right6")
public class Right6 extends Left6 {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.SIDE = Globals.Side.RIGHT;
        super.runOpMode();
    }
}
