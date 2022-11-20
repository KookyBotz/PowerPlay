package org.firstinspires.ftc.teamcode.opmode.test.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.common.freightfrenzy.Alliance;
//import org.firstinspires.ftc.teamcode.common.freightfrenzy.CommandBaseRobot;

import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystemcommands.subsystem.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
@Config
@TeleOp(name = "CommandBaseTest")
public class CommandBaseTest extends CommandOpMode {
//    private CommandBaseRobot robot;
//    private boolean intake = true;
//    private BooleanSupplier outtake;
//    private Consumer<Boolean> done;
//    private DoubleSupplier linkage, arm;
//
//    @Override
//    public void initialize() {
//        robot = new CommandBaseRobot(hardwareMap);
//
//        outtake = () -> gamepad1.right_bumper;
//        done = (a) -> intake = a;
//        linkage = () -> gamepad1.right_trigger;
//        arm = () -> gamepad1.left_trigger;
//
//        robot.turret.middle();
//        robot.arm.linkageIn();
//        robot.bucket.in();
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        robot.arm.loop();
//
//        if (gamepad1.a) {
//            schedule(new SharedCommand(robot, Alliance.RED, outtake, linkage, arm, done));
//        }
//    }a
    Robot robot;

    private ElapsedTime timer;
    private double loopTime = 0;

    boolean pDLB = false;
    boolean pDRB = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
        }

        robot.read();

        boolean dLB = gamepad1.left_bumper;
        if (dLB && !pDLB) {
            schedule(
                    new ClawCommand(robot, IntakeSubsystem.ClawState.CLOSED)
            );
        }
        pDLB = dLB;

        boolean dRB = gamepad1.right_bumper;
        if (dRB && !pDRB) {
            schedule(
                    new ClawCommand(robot, IntakeSubsystem.ClawState.OPEN)
            );
        }
        pDRB = dRB;

        CommandScheduler.getInstance().run();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        PhotonCore.EXPANSION_HUB.clearBulkCache();
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }
}
