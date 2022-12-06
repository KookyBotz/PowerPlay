package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.TeleopCycleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp(name = "Testing")
public class Testing extends OpMode {
    Robot robot;
    private double loopTime = 0;

    private boolean busy = false;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, true);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        robot.intake.extensionEncoder.resetEncoder();
        robot.lift.liftEncoder.resetEncoder();
        robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION);
        robot.intake.update(IntakeSubsystem.ClawState.OPEN);
        robot.intake.update(IntakeSubsystem.TurretState.INTAKE);
        robot.lift.update(LiftSubsystem.LatchState.UNLATCHED);
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        if (a && !busy) {
            busy = true;
            CommandScheduler.getInstance().schedule(new TeleopCycleCommand(robot, (b) -> busy = b));
        }

        robot.read();
        CommandScheduler.getInstance().run();
        robot.intake.loop();
        robot.lift.loop();
        robot.write();

        telemetry.addData("STATE: ", robot.intake.state);
        telemetry.addData("STATE: ", robot.lift.state);
        telemetry.addData("intake ", robot.intake.getPos());
        telemetry.addData("lift ", robot.lift.getPos());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}
