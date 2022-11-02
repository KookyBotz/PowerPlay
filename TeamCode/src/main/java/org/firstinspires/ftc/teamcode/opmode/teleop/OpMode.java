package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

@Config
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;

    public static double extensionPower = 0.1;
    public static double liftPower = 0.1;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setFourbar(0.6);
        robot.startIMUThread(this);
        robot.reset();
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        // Drivetrain
        Pose drive = new Pose(
                new Point(-gamepad1.left_stick_y,
                        gamepad1.left_stick_x).rotate(-robot.getAngle()),
                -gamepad1.right_stick_x
        );


        if (gamepad1.left_bumper) {
            robot.intake.extension.resetEncoder();
            robot.lift.lift.resetEncoder();
        }

        // Gamepad2
        if (gamepad2.dpad_up) {
            // manual extend
            robot.lift.lift.set(liftPower);
        } else if (gamepad2.dpad_down) {
            // manual retract
            robot.lift.lift.set(-liftPower);
        }

        if (gamepad2.dpad_left) {
            // intake extend
            robot.intake.extension.setTargetPosition(robot.intake.getPos() + 1);
        } else if (gamepad2.dpad_right) {
            // intake retract

        }

        if (gamepad2.right_bumper) {
            // rotate turret right
            // TODO revisit later
            Range.clip(0, 0, 0);
        } else if (gamepad2.left_bumper) {
            // rotate turret left
        }

        if (gamepad2.left_trigger > 0.3) {
            schedule(new OpenClawCommand(robot));
        } else if (gamepad2.right_trigger > 0.3) {
            schedule(new CloseClawCommand(robot));
        }

        double gamepad2_left_stick = gamepad2.left_stick_x;
        if (gamepad2_left_stick > 0.15 || gamepad2_left_stick < -0.15) {
            robot.intake.setFourbarFactor(gamepad2_left_stick);
        }

        robot.drivetrain.set(drive);
        robot.drivetrain.updateModules();

        robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();

        // Telemetry
        telemetry.addData("liftPos:", robot.lift.getPos());
        telemetry.addData("liftPow:", robot.lift.power);
        telemetry.addData("intakePos:", robot.intake.getPos());
        telemetry.addData("intakePow:", robot.intake.power);

        double loop = System.currentTimeMillis();
        telemetry.addData("hz ", 1000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        PhotonCore.EXPANSION_HUB.clearBulkCache();
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        robot.intake.extension.resetEncoder();
        robot.intake.extension.set(0);
        robot.lift.lift.resetEncoder();
        robot.lift.lift.set(0);
    }
}
