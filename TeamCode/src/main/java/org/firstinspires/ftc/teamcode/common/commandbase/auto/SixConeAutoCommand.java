package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.GrabPosition;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Locale;
import java.util.function.DoubleSupplier;

public class SixConeAutoCommand extends CommandBase {
    public int stackHeight = 5;
    private final GrabPosition[] GRAB_POSITIONS = new GrabPosition[]{
            new GrabPosition(570, 0, 0.173, 0.37, 0),
            new GrabPosition(555, 0, 0.14, 0.37, 0),
            new GrabPosition(550, 0, 0.105, 0.37, 0),
            new GrabPosition(550, 0, 0.075, 0.37, 0),
            new GrabPosition(560, 0, 0.04, 0.37, 0)
    };


    private final DoubleSupplier TIME_LEFT;

    public boolean canRetractIntake = false;
    public boolean canRetractDeposit = false;
    public boolean canDeposit = true;
    public boolean canIntake = false;
    public boolean inPosition = false;
    public boolean intaking = false;

    private String log = "init";

    private final RobotHardware robot;
    private final SwerveDrivetrain drive;
    private final IntakeSubsystem intake;
    private final LiftSubsystem lift;

    private CancelableGrabCommand GRAB_COMMAND;
    private CancelableDepositCommand DEPOSIT_COMMAND;
    private Command RETRACT_INTAKE_COMMAND;
    private Command RETRACT_DEPOSIT_COMMAND;

    public SixConeAutoCommand(RobotHardware robot, SwerveDrivetrain drivetrain, IntakeSubsystem intake, LiftSubsystem lift, DoubleSupplier time) {
        this.TIME_LEFT = time;
        this.robot = robot;
        this.drive = drivetrain;
        this.intake = intake;
        this.lift = lift;
    }

    @Override
    public void execute() {
        inPosition = drive.isLocked();

        // can we intake?
        if (inPosition && !canDeposit && !intaking && TIME_LEFT.getAsDouble() >= 4 && stackHeight > 0) canIntake = true;

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
        if (inPosition && canDeposit && TIME_LEFT.getAsDouble() >= 2.5) {
            log += "\n deposit";
            DEPOSIT_COMMAND = new CancelableDepositCommand(lift, LiftSubsystem.LiftState.HIGH, this);
            CommandScheduler.getInstance().schedule(DEPOSIT_COMMAND);
            canDeposit = false;
        }
    }

    @Override
    public boolean isFinished() {
        return TIME_LEFT.getAsDouble() < 2;
    }

    @Override
    public void end(boolean interrupted) {

    }

    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "CONES: %d, inPosition: %b, intaking: %b, canIntake: %b, canRetract: %b, canDeposit: %b",
                stackHeight, inPosition, intaking, canIntake, canRetractIntake, canDeposit) + "\n" + log;
    }
}
