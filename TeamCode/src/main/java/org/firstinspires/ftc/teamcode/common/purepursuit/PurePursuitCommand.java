package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.List;

public class PurePursuitCommand extends CommandBase {

    private Drivetrain drive;
    private List<Waypoints> waypoints;

    public PurePursuitCommand(Drivetrain drive, List<Waypoints> waypoints) {
        this.drive = drive;
        this.waypoints = waypoints;
    }
}
