package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeExtendCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Config
@TeleOp(name = "OpModeTest")
public class OpMode extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;
    private boolean fA = false;
    private boolean fB = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //robot.intake.openClaw();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        double loopTime2 = System.currentTimeMillis();

        // use fallimg edge dedteier
        boolean a = gamepad1.a;
        if (a && !fA) {
            schedule(new InstantCommand(() -> robot.lift.resetTimer())
            .alongWith(new InstantCommand(() -> robot.lift.setDVA(500, 500, 800))));
        }
        boolean fA = a;

        boolean b = gamepad1.b;
        if (b && !fB) {
            schedule(new InstantCommand(() -> robot.lift.setDVA(-500, -500, -800))
            .alongWith(new InstantCommand(() -> robot.lift.resetTimer())));
        }
        fB = b;

        //robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();

        telemetry.addData("u/s: ", loopTime2 - loopTime);
        telemetry.addData("target:", 135);
        telemetry.addData("curPos:", robot.lift.getPos());
        telemetry.update();

        loopTime = System.currentTimeMillis();
    }
}
