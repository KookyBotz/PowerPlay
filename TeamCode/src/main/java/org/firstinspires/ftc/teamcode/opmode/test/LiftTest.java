package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.command.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.commandgroups.OuttakeLevelCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class LiftTest extends OpMode {
    private Robot robot;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> System.out.println("driving"))
                        .alongWith(new OuttakeLevelCommand(robot, 10))
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
