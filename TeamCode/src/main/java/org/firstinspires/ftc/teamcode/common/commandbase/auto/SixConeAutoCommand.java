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
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Locale;
import java.util.function.DoubleSupplier;

public class SixConeAutoCommand extends CommandBase {

    private final Pose RISKY_CYCLE_POS = new Pose(-60.8, 3, 0.26 - Math.PI / 2);
    private final Pose RAM_POSE_START = new Pose(-40, 2, -Math.PI / 2);
    private final Pose RAM_POSE_END = new Pose(-68, 3, -Math.PI / 2);

    enum STATE {
        CYCLE, RAM, CYCLE2, RAM2, CYCLE3, PARK;

        public STATE next() {
            switch (this) {
                case CYCLE:
                    return RAM;
                case RAM:
                    return CYCLE2;
                case CYCLE2:
                    return RAM2;
                case RAM2:
                    return CYCLE3;
                default:
                    return PARK;
            }
        }
    }

    public STATE state = STATE.CYCLE;
    public ElapsedTime stateTime;

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
    public boolean ramming = false;
    public boolean entered = false;

    private String log = "init";

    private final RobotHardware robot;
    private final Localizer localizer;
    private final SwerveDrivetrain drive;
    private final IntakeSubsystem intake;
    private final LiftSubsystem lift;

    private CancelableGrabCommand GRAB_COMMAND;
    private CancelableDepositCommand DEPOSIT_COMMAND;
    private Command RETRACT_INTAKE_COMMAND;
    private Command RETRACT_DEPOSIT_COMMAND;

    public SixConeAutoCommand(RobotHardware robot, Localizer localizer, SwerveDrivetrain drivetrain, IntakeSubsystem intake, LiftSubsystem lift, DoubleSupplier time) {
        this.TIME_LEFT = time;
        this.robot = robot;
        this.localizer = localizer;
        this.drive = drivetrain;
        this.intake = intake;
        this.lift = lift;
    }

    @Override
    public void initialize() {
        stateTime = new ElapsedTime();
    }

    @Override
    public void execute() {
        inPosition = drive.isLocked();

        // reset state timer
        if (inPosition) stateTime.reset();
        if (ramming) stateTime.reset();

        // definitely not chilling
        if ((state == STATE.CYCLE || state == STATE.CYCLE2 || state == STATE.CYCLE3) && stateTime.seconds() > 3) {
            log += "\n yikes, entering" + state.next().toString();
            state = state.next();
            entered = true;
            stateTime.reset();
        }

        // haha auto go brr
        if ((state == STATE.RAM || state == STATE.RAM2) && !ramming && (entered || stateTime.seconds() > 3) && TIME_LEFT.getAsDouble() > 6) {
            stateTime.reset();
            log += "entering " + state;
            ramming = true;
            entered = false;
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> PositionLockCommand.setTargetPose(new Pose())),
                            new PositionCommand(drive, localizer, RAM_POSE_START, 0, 1500, robot.getVoltage()),
                            new PositionCommand(drive, localizer, RAM_POSE_END, 0, 1500, robot.getVoltage()),
                            new PositionCommand(drive, localizer, RISKY_CYCLE_POS, 0, 1000, robot.getVoltage()),
                            new InstantCommand(() -> ramming = false),
                            new InstantCommand(() -> PositionLockCommand.setTargetPose(RISKY_CYCLE_POS))
                    )
            );
        }

        // we good again
        if ((state == STATE.RAM || state == STATE.RAM2) && inPosition && PositionLockCommand.getTargetPose() == RISKY_CYCLE_POS) {
            state = state.next();
            stateTime.reset();
        }

        // chilling
        if (state == STATE.CYCLE || state == STATE.CYCLE2 || state == STATE.CYCLE3) {
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
            if (inPosition && canDeposit && TIME_LEFT.getAsDouble() >= 3) {
                log += "\n deposit";
                DEPOSIT_COMMAND = new CancelableDepositCommand(lift, this);
                canDeposit = false;
                if (stackHeight == 0) {
                    CommandScheduler.getInstance().schedule(
                            new WaitUntilCommand(() -> TIME_LEFT.getAsDouble() <= 3)
                                    .andThen(DEPOSIT_COMMAND)
                    );
                } else {
                    CommandScheduler.getInstance().schedule(DEPOSIT_COMMAND);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return TIME_LEFT.getAsDouble() < 2 || state == STATE.PARK;
    }

    @Override
    public void end(boolean interrupted) {
        log += "\n parking!";
        if (canRetractIntake) CommandScheduler.getInstance().schedule(new RetractIntakeCommand(intake));
        if (canRetractDeposit) CommandScheduler.getInstance().schedule(new RetractDepositCommand(lift));
        PositionLockCommand.setTargetPose(new Pose());
        CommandScheduler.getInstance().schedule(new PositionCommand(drive, localizer, new Pose(-52, 3, 0), 0, 2000, robot.getVoltage()));
    }

    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "CONES: %d, STATE: %s, StateTime: %.4f",
                stackHeight, state.toString(), stateTime == null ? 0 : stateTime.seconds()) + "\n" + log;
    }
}
