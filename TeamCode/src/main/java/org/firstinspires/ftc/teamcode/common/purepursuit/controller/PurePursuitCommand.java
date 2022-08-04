package org.firstinspires.ftc.teamcode.common.purepursuit.controller;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;

import java.util.List;

public class PurePursuitCommand extends CommandBase {

    private Drivetrain drive;
    private List<Waypoint> waypoints;

    public PurePursuitCommand(Drivetrain drive, List<Waypoint> waypoints) {
        this.drive = drive;
        this.waypoints = waypoints;
    }
}
