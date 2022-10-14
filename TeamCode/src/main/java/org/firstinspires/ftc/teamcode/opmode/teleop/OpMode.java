package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

        // use fallimg edge dedteier
        boolean a = gamepad1.a;
        if (a && !fA) {
            schedule(new InstantCommand(() -> robot.lift.resetTimer())
            .alongWith(new InstantCommand(() -> robot.lift.setDVA(500, 1500, 7500))));
        }
        boolean fA = a;

        boolean b = gamepad1.b;
        if (b && !fB) {
            schedule(new InstantCommand(() -> robot.lift.setDVA(-500, -1500, -7500))
            .alongWith(new InstantCommand(() -> robot.lift.resetTimer())));
        }
        fB = b;

        //robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();

        telemetry.addData("curPos:", robot.lift.getPos());
        telemetry.addData("curPow:", robot.lift.power);
        telemetry.addData("targetPos", robot.lift.funcs[0]);
        telemetry.addData("targetVel", robot.lift.funcs[1]);
        telemetry.addData("targetAcc", robot.lift.funcs[2]);
        telemetry.addData("draw", robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS));

        double loop = System.currentTimeMillis();
        telemetry.addData("hz ", 1000 / (loop - loopTime));

        telemetry.update();

        loopTime = loop;
    }
}
