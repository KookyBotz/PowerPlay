package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection.*;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SmartParkCommand extends SequentialCommandGroup {
    public SmartParkCommand(SwerveDrivetrain drive, Localizer localizer, RobotHardware robot, ParkingPosition parkingPosition, boolean smartPark) {
        super(
                new PositionCommand(drive, localizer, new Pose(-28, 3, Math.PI), 0, 2000, robot.getVoltage()),
                new ConditionalCommand(
                        new PositionCommand(drive, localizer, ((parkingPosition == ParkingPosition.CENTER) ? new Pose(-28, 3, Math.PI) : (parkingPosition == ParkingPosition.LEFT ? new Pose(-28, 16, Math.PI) : new Pose(-28, -13, Math.PI))), 0, 2000, robot.getVoltage()),
                        new WaitCommand(0),
                        () -> smartPark
                )
        );
    }

}
