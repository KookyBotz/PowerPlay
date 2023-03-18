package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Constants;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;

public class ParkSequence extends SequentialCommandGroup {
    public ParkSequence(Robot robot, HardwareMap hardwareMap, Constants.Side currentSide, SleeveDetection.ParkingPosition position) {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new PositionCommand(robot.drivetrain, robot.localizer, new Pose(31.5, -1, Math.PI / 2), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                                new WaitCommand(500),
                                new PositionCommand(robot.drivetrain, robot.localizer,
                                        position == SleeveDetection.ParkingPosition.LEFT ? new Pose(29.5, 24, Math.PI / 2) :
                                                position == SleeveDetection.ParkingPosition.CENTER ? new Pose(29.5, -1, Math.PI / 2) :
                                                        new Pose(29.5, -25, Math.PI / 2), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage())
                        ), new PositionCommand(robot.drivetrain, robot.localizer,
                        position == SleeveDetection.ParkingPosition.LEFT ? new Pose(56.5, -45.5, Math.PI / 2) :
                                position == SleeveDetection.ParkingPosition.CENTER ? new Pose(56.5, -70.5, Math.PI / 2) :
                                        new Pose(56.5, -94.5, Math.PI / 2), 2000, 2000, hardwareMap.voltageSensor.iterator().next().getVoltage()),
                        () -> (currentSide == Constants.Side.LEFT)
                )
        );
    }
}
