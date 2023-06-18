package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import javax.annotation.concurrent.GuardedBy;

public class RobotHardware {
    public MotorEx liftLeft;
    public MotorEx liftRight;
    public MotorEx extension;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public Servo claw;
    public Servo turret;
    public Servo pivot;
    public Servo fourbarLeft;
    public Servo fourbarRight;
    public Servo latch;

    public CRServo frontLeftServo;
    public CRServo frontRightServo;
    public CRServo backLeftServo;
    public CRServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder intakeEncoder;
    public Motor.Encoder parallelPod;
    public Motor.Encoder perpindicularPod;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double voltage = 0.0;
    private ElapsedTime voltageTimer;

    public OpenCvCamera backCamera;

    public DigitalChannel clawSensor;
    public SleeveDetection sleeveDetection;

    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        if (Globals.USING_IMU) {
            synchronized (imuLock) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
        }

        voltageTimer = new ElapsedTime();

        liftLeft = new MotorEx(hardwareMap, "motorLiftLeft");
        liftLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight = new MotorEx(hardwareMap, "motorLiftRight");
        liftRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension = new MotorEx(hardwareMap, "extension");
        extension.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        claw = hardwareMap.get(Servo.class, "claw");
        turret = hardwareMap.get(Servo.class, "turret");
        pivot = hardwareMap.get(Servo.class, "pivot");
        fourbarLeft = hardwareMap.get(Servo.class, "fourbarLeft");
        fourbarRight = hardwareMap.get(Servo.class, "fourbarRight");

        latch = hardwareMap.get(Servo.class, "latch");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        liftEncoder = new MotorEx(hardwareMap, "frontLeftMotor").encoder;
        liftEncoder.setDirection(Motor.Direction.REVERSE);
        intakeEncoder = new MotorEx(hardwareMap, "frontRightMotor").encoder;

        parallelPod = new MotorEx(hardwareMap, "backLeftMotor").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        perpindicularPod = new MotorEx(hardwareMap, "backRightMotor").encoder;
        perpindicularPod.setDirection(Motor.Direction.REVERSE);

        clawSensor = hardwareMap.get(DigitalChannel.class, "clawSensor");

        if (Globals.AUTO) {
            sleeveDetection = new SleeveDetection();
            backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
            backCamera.setPipeline(sleeveDetection);
            backCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    backCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void loop(Pose drive, SwerveDrivetrain drivetrain, IntakeSubsystem intake, LiftSubsystem lift) {
        try {
            if (drive != null) {
                drivetrain.set(drive);
            }
            drivetrain.updateModules();
        } catch (Exception ignored) {
        }
        try {
            intake.loop2();
        } catch (Exception ignored) {
        }
        try {
            lift.loop();
        } catch (Exception ignored) {
        }

        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
    }

    public void read(SwerveDrivetrain drivetrain, IntakeSubsystem intake, LiftSubsystem lift) {
        try {
            intake.read();
        } catch (Exception ignored) {
        }
        try {
            lift.read();
        } catch (Exception ignored) {
        }
        try {
            drivetrain.read();

        } catch (Exception ignored) {
        }
    }

    public void write(SwerveDrivetrain drivetrain, IntakeSubsystem intake, LiftSubsystem lift) {
        try {
            intake.write();
        } catch (Exception ignored) {
        }
        try {
            lift.write();
        } catch (Exception ignored) {
        }
        try {
            drivetrain.write();

        } catch (Exception ignored) {
        }
    }

    public void reset() {
        try {
            intakeEncoder.reset();
            liftEncoder.reset();
            parallelPod.reset();
            perpindicularPod.reset();
        } catch (Exception e) {
        }
        imuOffset = imu.getAngularOrientation().firstAngle;
    }

    public void clearBulkCache() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getAngularOrientation().firstAngle;
                    }
                }
            });
            imuThread.start();
        }
    }

    public void stopCameraStream() {
        backCamera.closeCameraDeviceAsync(() -> System.out.println("Stopped Back Camera"));
    }

    public double getVoltage() {
        return voltage;
    }

    public void zero() {
        extension.motor.setPower(0.01);
        liftLeft.motor.setPower(0.01);
        liftRight.motor.setPower(-0.01);
    }
}
