package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class OpMode extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        //robot.intake.openClaw();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        double loopTime2 = System.currentTimeMillis();

        if (gamepad1.a) {
            schedule(new InstantCommand(() -> robot.intake.resetTimer())
            .alongWith(new InstantCommand(() -> robot.intake.setExtension(135))));
        }

        telemetry.addData("u/s: ", loopTime2 - loopTime);
        telemetry.update();

        loopTime = System.currentTimeMillis();
    }
}
