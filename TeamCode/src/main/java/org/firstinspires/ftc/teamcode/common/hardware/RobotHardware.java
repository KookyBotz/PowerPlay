package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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
    public Motor.Encoder extensionEncoder;
    public Motor.Encoder horizontalPod;
    public Motor.Encoder lateralPod;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public SwerveDrivetrain drivetrain;
    public Localizer localizer;

    public DigitalChannel clawSensor;
    public DigitalChannel depositSensor;

    public OpenCvCamera camera;

    public VoltageSensor voltageSensor;

    public OpenCvPipeline pipeline;

    public static RobotHardware instance = new RobotHardware();

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        try {
            if (!Globals.USING_IMU) throw new Exception();
            synchronized (imuLock) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
        } catch (Exception e) {
            imu = null;
        }

        try {
            liftLeft = new MotorEx(hardwareMap, "liftLeft");
        } catch (Exception e) {
            liftLeft = null;
        }

        try {
            liftRight = new MotorEx(hardwareMap, "liftRight");
        } catch (Exception e) {
            liftRight = null;
        }

        try {
            extension = new MotorEx(hardwareMap, "extension");
        } catch (Exception e) {
            extension = null;
        }

        try {
            frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        } catch (Exception e) {
            frontLeftMotor = null;
        }

        try {
            frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        } catch (Exception e) {
            frontRightMotor = null;
        }

        try {
            backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        } catch (Exception e) {
            backLeftMotor = null;
        }

        try {
            backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        } catch (Exception e) {
            backRightMotor = null;
        }

        try {
            claw = hardwareMap.get(Servo.class, "claw");
        } catch (Exception e) {
            claw = null;
        }

        try {
            turret = hardwareMap.get(Servo.class, "turret");
        } catch (Exception e) {
            turret = null;
        }

        try {
            pivot = hardwareMap.get(Servo.class, "pivot");
        } catch (Exception e) {
            pivot = null;
        }

        try {
            fourbarLeft = hardwareMap.get(Servo.class, "fourbarLeft");
        } catch (Exception e) {
            fourbarLeft = null;
        }

        try {
            fourbarRight = hardwareMap.get(Servo.class, "fourbarRight");
        } catch (Exception e) {
            fourbarRight = null;
        }

        try {
            latch = hardwareMap.get(Servo.class, "latch");
        } catch (Exception e) {
            latch = null;
        }

        try {
            frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        } catch (Exception e) {
            frontLeftServo = null;
        }

        try {
            frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        } catch (Exception e) {
            frontRightServo = null;
        }

        try {
            backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        } catch (Exception e) {
            backLeftServo = null;
        }

        try {
            backRightServo = hardwareMap.get(CRServo.class, "backRightServo");
        } catch (Exception e) {
            backRightServo = null;
        }

        try {
            frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        } catch (Exception e) {
            frontLeftEncoder = null;
        }

        try {
            frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        } catch (Exception e) {
            frontRightEncoder = null;
        }

        try {
            backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        } catch (Exception e) {
            backLeftEncoder = null;
        }

        try {
            backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        } catch (Exception e) {
            backRightEncoder = null;
        }

        try {
            liftEncoder = new MotorEx(hardwareMap, "liftEncoder").encoder;
        } catch (Exception e) {
            liftEncoder = null;
        }

        try {
            horizontalPod = new MotorEx(hardwareMap, "horizontalPod").encoder;
        } catch (Exception e) {
            horizontalPod = null;
        }

        try {
            lateralPod = new MotorEx(hardwareMap, "lateralPod").encoder;
        } catch (Exception e) {
            lateralPod = null;
        }

        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, (Globals.SIDE.equals(Globals.Side.LEFT)) ? "WebcamLeft" : "WebcamRight"), cameraMonitorViewId);
            pipeline = new SleeveDetection();
        } catch (Exception e) {
            camera = null;
            pipeline = null;
        }

        try {
            clawSensor = hardwareMap.get(DigitalChannel.class, "clawSensor");
        } catch (Exception e) {
            clawSensor = null;
        }

        try {
            depositSensor = hardwareMap.get(DigitalChannel.class, "depositSensor");
        } catch (Exception e) {
            depositSensor = null;
        }

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    public void loop(Pose drive) {
        drivetrain.set(drive);
        drivetrain.updateModules();
        intake.loop();
        lift.loop();
    }

    public void read() {
        intake.read();
        lift.read();
        drivetrain.read();
    }

    public void write() {
        intake.write();
        lift.write();
        drivetrain.write();
    }

    public void reset() {
        extensionEncoder.reset();
        liftEncoder.reset();
    }

    public void clearBulkCache() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    public double getAngle() {
        return imuAngle;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            SwerveDrivetrain.imuOffset = -Math.PI / 2;
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = -imu.getAngularOrientation().firstAngle;
                    }
                }
            });
            imuThread.start();
        }
    }

    public void cameraInit() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}
