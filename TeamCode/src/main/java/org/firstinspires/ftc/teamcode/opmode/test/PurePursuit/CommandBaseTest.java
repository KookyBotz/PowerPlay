package org.firstinspires.ftc.teamcode.opmode.test.PurePursuit;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.common.freightfrenzy.Alliance;
//import org.firstinspires.ftc.teamcode.common.freightfrenzy.CommandBaseRobot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
@Disabled
@TeleOp
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
//    }
    @Override
    public void initialize() {}

    @Override
    public void run() {}
}
