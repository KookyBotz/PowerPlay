package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.Left6.getModifier;

import java.util.Locale;
import java.util.function.DoubleSupplier;

public class SixConeAutoCommand extends CommandBase {
    public int stackHeight = 5;
    private static final GrabPosition[] GRAB_POSITIONS = new GrabPosition[]{
            new GrabPosition(570, 0, 0.173, 0.37, 0),
            new GrabPosition(555, 0, 0.14, 0.37, 0),
            new GrabPosition(552, 0, 0.115, 0.37, 0),
            new GrabPosition(552, 0, 0.08, 0.37, 0),
            new GrabPosition(555, 0, 0.045, 0.37, 0)
    };

    private final DoubleSupplier TIME_LEFT;

    public boolean canRetractIntake = false;
    public boolean canRetractDeposit = false;
    public boolean canDeposit = true;
    public boolean canIntake = false;
    public boolean inPosition = false;
    public boolean intaking = false;
    public boolean finished = false;

    private String log = "init";

    private final RobotHardware robot;
    private final Localizer localizer;
    private final SwerveDrivetrain drive;
    private final IntakeSubsystem intake;
    private final LiftSubsystem lift;
    private final ParkingPosition sleevePosition;

    private CancelableGrabCommand GRAB_COMMAND;
    private CancelableDepositCommand DEPOSIT_COMMAND;
    private Command RETRACT_INTAKE_COMMAND;
    private Command RETRACT_DEPOSIT_COMMAND;

    public SixConeAutoCommand(RobotHardware robot, Localizer localizer, SwerveDrivetrain drivetrain, IntakeSubsystem intake, LiftSubsystem lift, DoubleSupplier time, ParkingPosition sleevePosition) {
        this.TIME_LEFT = time;
        this.robot = robot;
        this.localizer = localizer;
        this.drive = drivetrain;
        this.intake = intake;
        this.lift = lift;
        this.sleevePosition = sleevePosition;
    }

    @Override
    public void execute() {
        inPosition = drive.isLocked();

        // can we intake?
        if (inPosition && !canDeposit && !intaking && TIME_LEFT.getAsDouble() >= 4 && stackHeight > 0)
            canIntake = true;

        // if we can, grab a cone
        if (canIntake) {
            log += "\n intake";
            intaking = true;
            canIntake = false;
            GRAB_COMMAND = new CancelableGrabCommand(lift, intake, GRAB_POSITIONS[5 - stackHeight], this);
            CommandScheduler.getInstance().schedule(GRAB_COMMAND);
        }

        // got pushed, need to retract intake asap (if we can)
        if (!inPosition && canRetractIntake) {
            log += "\n pushed, retracting intake";
            canIntake = false;
            canRetractIntake = false;
            intaking = false;
            GRAB_COMMAND.cancel();
            GRAB_COMMAND = null;
            RETRACT_INTAKE_COMMAND = new RetractIntakeCommand(intake);
            CommandScheduler.getInstance().schedule(RETRACT_INTAKE_COMMAND);
        }

        // got pushed, need to retract deposit asap (if we can)
        if (!inPosition && canRetractDeposit) {
            log += "\n pushed, retracting deposit";
            canDeposit = true;
            canRetractDeposit = false;
            DEPOSIT_COMMAND.cancel();
            DEPOSIT_COMMAND = null;
            RETRACT_DEPOSIT_COMMAND = new RetractDepositCommand(lift);
            CommandScheduler.getInstance().schedule(RETRACT_DEPOSIT_COMMAND);
        }

        //deposit
        if (inPosition && canDeposit && TIME_LEFT.getAsDouble() >= 4) {
            log += "\n deposit";
            DEPOSIT_COMMAND = new CancelableDepositCommand(lift, this);
            canDeposit = false;
            if (stackHeight == 0) {
                CommandScheduler.getInstance().schedule(
                        new WaitUntilCommand(() -> TIME_LEFT.getAsDouble() <= 4)
                                .andThen(DEPOSIT_COMMAND)
                );
            } else {
                CommandScheduler.getInstance().schedule(DEPOSIT_COMMAND);
            }
        }

        if (TIME_LEFT.getAsDouble() < 3) finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        log += "\n parking!";
        if (canRetractIntake)
            CommandScheduler.getInstance().schedule(new RetractIntakeCommand(intake));
        if (canRetractDeposit)
            CommandScheduler.getInstance().schedule(new RetractDepositCommand(lift));
        PositionLockCommand.setTargetPose(new Pose());

        if (sleevePosition == ParkingPosition.CENTER) {
            CommandScheduler.getInstance().schedule(new PositionCommand(drive, localizer, new Pose(-52, 3 * getModifier(), 0), 0, 1000, robot.getVoltage()));
        }
        if (sleevePosition == ParkingPosition.LEFT) {
            CommandScheduler.getInstance().schedule(
                    new PositionCommand(drive, localizer, new Pose(-52, 3 * getModifier(), 0), 0, 1000, robot.getVoltage())
                            .andThen(new PositionCommand(drive, localizer, new Pose(-52, 27 * getModifier(), 0), 0, 1000, robot.getVoltage()))
            );
        }
        if (sleevePosition == ParkingPosition.RIGHT) {
            CommandScheduler.getInstance().schedule(
                    new PositionCommand(drive, localizer, new Pose(-52, 3 * getModifier(), 0), 0, 1000, robot.getVoltage())
                            .andThen(new PositionCommand(drive, localizer, new Pose(-52, -21 * getModifier(), 0), 0, 1000, robot.getVoltage()))
            );
        }
    }


    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "CONES: %d, time: %.2f",
                stackHeight, TIME_LEFT.getAsDouble()) + "\n" + log;
    }
}
