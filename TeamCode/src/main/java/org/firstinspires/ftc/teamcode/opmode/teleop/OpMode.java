package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.auto.CycleCommand;
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

    boolean pDup = false;
    boolean pDLB = false;
    boolean pDRB = false;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setFourbar(0.6);
        robot.reset();
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        robot.read();
//        double loop = System.nanoTime();
//        telemetry.addData("READ hz ", 1000000000 / (loop - loopTime));
//        loopTime = loop;

        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        // Drivetrain
        Pose drive = new Pose(
                new Point(Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3),
                        Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3)).rotate(0),
                Math.pow(-gamepad1.right_stick_x, 3)
        );

        if (gamepad1.left_bumper) {
            robot.intake.extension.resetEncoder();
            robot.lift.lift.resetEncoder();
        }

        // Gamepad2
        if (gamepad2.dpad_up) {
            robot.lift.setSlideFactor(1);
        } else if (gamepad2.dpad_down) {
            robot.lift.setSlideFactor(-1);
        }

        if (gamepad2.dpad_left) {
            // intake extend
            //robot.intake.extension.setTargetPosition(robot.intake.getPos() + 1);
        } else if (gamepad2.dpad_right) {
            // intake retract
        }

        if (gamepad2.left_trigger > 0.3) {
            schedule(new OpenClawCommand(robot));
        } else if (gamepad2.right_trigger > 0.3) {
            schedule(new CloseClawCommand(robot));
        }

        double gamepad2_left_stick_y = gamepad2.left_stick_y;
        if (Math.abs(gamepad2_left_stick_y) > 0.15) {
            robot.intake.setFourbarFactor(gamepad2_left_stick_y);
        }

        double gamepad2_left_stick_x = gamepad2.left_stick_x;
        if (Math.abs(gamepad2_left_stick_x) > 0.15) {
            robot.intake.setSlideFactor(gamepad2_left_stick_x);
        }

        double gamepad2_right_stick = gamepad2.right_stick_x;
        if (Math.abs(gamepad2_right_stick) > 0.15) {
            robot.intake.setTurretFactor(gamepad2_right_stick);
        }

        boolean dLB = gamepad2.left_bumper;
        boolean dRB = gamepad2.right_bumper;
        if (dLB && !pDLB) {
            schedule(new InstantCommand(() -> robot.intake.intakeTurret()),
                    new InstantCommand(() -> robot.intake.extendForebar()),
                    new InstantCommand(() -> robot.intake.openClaw()));
        } else if (dRB && !pDRB) {
//            if (robot.lift.getPos() < 10 && robot.intake.getPos() < 10) {
//                schedule(new InstantCommand(() -> robot.intake.closeClaw()),
//                        new WaitCommand(500),
//                        new InstantCommand(() -> robot.intake.depositTurret()),
//                        new InstantCommand(() -> robot.intake.closeForebar()));
//            }
            schedule(new IntakeRetractCommand(robot));
        }
        pDRB = dRB;
        pDLB = dLB;

        if (gamepad2.a) {
            schedule(new LiftCommand(robot, 150, 400, 1500));
        } else if (gamepad2.x) {
            schedule(new LiftCommand(robot, 385, 400, 1500));
        } else if (gamepad2.y) {
            schedule(new LiftCommand(robot, 610, 700, 3000));
        } else if (gamepad2.b) {
            schedule(new InstantCommand(() -> robot.lift.newProfile(-10, 3500, 8500)));
        }

//        boolean dup = gamepad2.dpad_up;
//        if (dup && !pDup) {
//            schedule(new CycleCommand(robot));
//        }
//        pDup = dup;

//        loop = System.nanoTime();
//        telemetry.addData("schedule HZ ", 1000000000 / (loop - loopTime));
//        loopTime = loop;

        robot.drivetrain.set(drive);
        robot.drivetrain.updateModules();
//        loop = System.nanoTime();
//        telemetry.addData("update HZ ", 1000000000 / (loop - loopTime));
//        loopTime = loop;

        robot.intake.loop();
        robot.lift.loop();
        CommandScheduler.getInstance().run();
//        loop = System.nanoTime();
//        telemetry.addData("command/loop HZ ", 1000000000 / (loop - loopTime));
//        loopTime = loop;

        robot.write();
//        loop = System.nanoTime();
//        telemetry.addData("write HZ ", 1000000000 / (loop - loopTime));
//        loopTime = loop;

        // Telemetry
        telemetry.addData("liftPos:", robot.lift.getPos());
        telemetry.addData("liftPow:", robot.lift.power);
        telemetry.addData("intakePos:", robot.intake.getPos());
        telemetry.addData("intakePow:", robot.intake.power);
        telemetry.addData("intakeTarget:", robot.intake.targetPosition);
        telemetry.addData("velocity:", robot.intake.curState.getV());
        telemetry.addData("state:", robot.intake.curState.getV() == 0);
        telemetry.addData("speed multiplier:", speedMultiplier);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
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
