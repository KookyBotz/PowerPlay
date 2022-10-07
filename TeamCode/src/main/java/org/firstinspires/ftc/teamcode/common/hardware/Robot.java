package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

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

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new SwerveDrivetrain(hardwareMap);

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        MotorEx extension = new MotorEx(hardwareMap, "extension");
        Servo   barLeft   = hardwareMap.get(Servo.class, "forebarLeft"),
                barRight  = hardwareMap.get(Servo.class, "forebarRight"),
                claw      = hardwareMap.get(Servo.class, "claw"),
                turret    = hardwareMap.get(Servo.class, "turret");
        intake = new IntakeSubsystem(extension, barLeft, barRight, claw, turret);

        DcMotorEx lift = new DcMotorEx(hardwareMap, "lift") {
            @Override
            public void setMotorEnable() {

            }

            @Override
            public void setMotorDisable() {

            }

            @Override
            public boolean isMotorEnabled() {
                return false;
            }

            @Override
            public void setVelocity(double angularRate) {

            }

            @Override
            public void setVelocity(double angularRate, AngleUnit unit) {

            }

            @Override
            public double getVelocity() {
                return 0;
            }

            @Override
            public double getVelocity(AngleUnit unit) {
                return 0;
            }

            @Override
            public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {

            }

            @Override
            public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {

            }

            @Override
            public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {

            }

            @Override
            public void setPositionPIDFCoefficients(double p) {

            }

            @Override
            public PIDCoefficients getPIDCoefficients(RunMode mode) {
                return null;
            }

            @Override
            public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
                return null;
            }

            @Override
            public void setTargetPositionTolerance(int tolerance) {

            }

            @Override
            public int getTargetPositionTolerance() {
                return 0;
            }

            @Override
            public double getCurrent(CurrentUnit unit) {
                return 0;
            }

            @Override
            public double getCurrentAlert(CurrentUnit unit) {
                return 0;
            }

            @Override
            public void setCurrentAlert(double current, CurrentUnit unit) {

            }

            @Override
            public boolean isOverCurrent() {
                return false;
            }

            @Override
            public MotorConfigurationType getMotorType() {
                return null;
            }

            @Override
            public void setMotorType(MotorConfigurationType motorType) {

            }

            @Override
            public DcMotorController getController() {
                return null;
            }

            @Override
            public int getPortNumber() {
                return 0;
            }

            @Override
            public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

            }

            @Override
            public ZeroPowerBehavior getZeroPowerBehavior() {
                return null;
            }

            @Override
            public void setPowerFloat() {

            }

            @Override
            public boolean getPowerFloat() {
                return false;
            }

            @Override
            public void setTargetPosition(int position) {

            }

            @Override
            public int getTargetPosition() {
                return 0;
            }

            @Override
            public boolean isBusy() {
                return false;
            }

            @Override
            public int getCurrentPosition() {
                return 0;
            }

            @Override
            public void setMode(RunMode mode) {

            }

            @Override
            public RunMode getMode() {
                return null;
            }

            @Override
            public void setDirection(Direction direction) {

            }

            @Override
            public Direction getDirection() {
                return null;
            }

            @Override
            public void setPower(double power) {

            }

            @Override
            public double getPower() {
                return 0;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return null;
            }

            @Override
            public String getConnectionInfo() {
                return null;
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return imuAngle;
    }
}
