package org.firstinspires.ftc.teamcode.common.hardware;

import android.content.Context;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

import javax.annotation.concurrent.GuardedBy;

/**
 * The robot class, containing all useful fields and subsystems within the robot.
 */
public class Robot {
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public final BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;

    public SwerveDrivetrain drivetrain;
    public Localizer localizer;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;

    public Motor.Encoder horizontalEncoder, lateralEncoder;

    private boolean isAuto = false;

    /**
     * Robot constructor, holding several subsystem classes within itself
     *
     * @param hardwareMap {@link HardwareMap}
     * @param isAuto
     */
    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        this.isAuto = isAuto;
        drivetrain = new SwerveDrivetrain(hardwareMap);

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }


        horizontalEncoder = new MotorEx(hardwareMap, "rightFrontMotor").encoder;
        lateralEncoder = new MotorEx(hardwareMap, "leftFrontMotor").encoder;

        intake = new IntakeSubsystem(hardwareMap, isAuto);

        lift = new LiftSubsystem(hardwareMap, isAuto);
    }

    /**
     * Generic robot constructor implementation, without an isAuto parameter
     *
     * @param hardwareMap
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * Starts imu thread
     * @param opMode our current opMode
     */
    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = -imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

    /**
     * Gets the current orientation of the robot
     *
     * @return The imu angle
     */
    public double getAngle() {
        return imuAngle;
    }

    /**
     * Resets the lift and intake slide encoders.
     */
    public void reset() {
        lift.liftEncoder.resetEncoder();
        intake.extensionEncoder.resetEncoder();
    }

    /**
     * Bulk caches every encoder position on the robot.
     */
    public void read() {
        intake.read();
        lift.read();
        for (SwerveModule module : drivetrain.modules) {
            module.read();
        }
    }

    /**
     * Bulk writes to all motors within the robot.
     */
    public void write() {
        intake.write();
        lift.write();
        if (this.isAuto) {
            drivetrain.writeAuto();
        } else {
            drivetrain.write();
        }
    }

    /**
     * Writes to a text file, containing all subsystem encoder positions and the IMU's last angle.
     * For use in TeleOp, for field centric drive.
     */
    public void writeFile() {
        FileInterface.clear();
        FileInterface.write(FileInterface.IMU, String.valueOf(getAngle()));
        FileInterface.write(FileInterface.INTAKE, String.valueOf(intake.getPos()));
        FileInterface.write(FileInterface.LIFT, String.valueOf(lift.getPos()));
    }

    /**
     * Reads from a text file, containing all subsystem encoder positions and the IMU's last angle.
     * For use in TeleOp, for field centric drive.
     */
    public void readFile() {
        intake.offset = Integer.parseInt(FileInterface.read(FileInterface.INTAKE));
        lift.offset = Integer.parseInt(FileInterface.read(FileInterface.LIFT));
        SwerveDrivetrain.imuOff = -1.75;
    }
}
