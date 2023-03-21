package org.firstinspires.ftc.teamcode.opmode.test.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.drive.drive.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@Config
@TeleOp(name = "IntakeTest")
public class IntakeTest extends CommandOpMode {
    Robot robot;
    private Servo barLeft, barRight;
    private Servo claw, turret;
    private Servo pivot;
    private DigitalChannel clawSensor;
    boolean rightBumper2 = false;
    boolean lastLT = false;

    public static double fourbarPosition = 0;
    public static double clawPosition = 0;
    public static double turretPosition = 0;
    public static double pivotPosition = 0;

    private boolean amogus = false;

    public static double offset = 0;
    private boolean transfer = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        this.barLeft = hardwareMap.get(Servo.class, "fourbarLeft");
        this.barRight = hardwareMap.get(Servo.class, "fourbarRight");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.turret = hardwareMap.get(Servo.class, "turret");
        this.pivot = hardwareMap.get(Servo.class, "pivot");
        this.clawSensor = hardwareMap.get(DigitalChannel.class, "clawSensor");
        clawSensor.setMode(DigitalChannel.Mode.INPUT);
        robot = new Robot(hardwareMap, false);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        barLeft.setPosition(0.385);
        barRight.setPosition(1 - 0.385);
        pivot.setPosition(0.545);
        turret.setPosition(0.728);
    }

    @Override
    public void run() {
        if (!amogus) {
//            robot.startIMUThread(this);
//            SwerveDrivetrain.imuOff = -Math.PI / 2;
            amogus = !amogus;
        }
        robot.read();

        double speedMultiplier = 1 - 0.75 * gamepad1.right_trigger;
        // Drivetrain
        Pose drive = new Pose(
                new Point((Math.pow(Math.abs(gamepad1.left_stick_y) > 0.02 ? gamepad1.left_stick_y : 0, 3)),
                        (Math.pow(-(Math.abs(gamepad1.left_stick_x) > 0.02 ? gamepad1.left_stick_x : 0), 3))).rotate(0),
                (Math.pow(-gamepad1.right_stick_x, 3)) * speedMultiplier
        );

        barLeft.setPosition(fourbarPosition);
        barRight.setPosition(1 - fourbarPosition);
        pivot.setPosition(pivotPosition);

        if (gamepad1.left_bumper) {
            SwerveDrivetrain.imuOffset = robot.getAngle() - Math.PI;
        }

        if (gamepad2.a) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> pivot.setPosition(0.4)),
                            new InstantCommand(() -> turret.setPosition(0.728)),
                            new InstantCommand(() -> barLeft.setPosition(0.359)),
                            new InstantCommand(() -> barRight.setPosition(1 - 0.359))
                    )
            );
        }

        if (gamepad2.b) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> pivot.setPosition(0.545)),
                            new InstantCommand(() -> turret.setPosition(0.728)),
                            new InstantCommand(() -> barLeft.setPosition(0.075)),
                            new InstantCommand(() -> barRight.setPosition(1 - 0.075))
                    )
            );
        }

        if (gamepad2.left_bumper) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> barLeft.setPosition(0.385)),
                            new InstantCommand(() -> barRight.setPosition(1 - 0.385)),
                            new InstantCommand(() -> pivot.setPosition(0.545)),
                            new InstantCommand(() -> turret.setPosition(0.449))
                    )
            );
        }

        // USE FSM, check if the last state (new variable that will need to be stored)
        // was intaking state
        boolean rightBumper = gamepad2.right_bumper;
        if (!rightBumper2 && rightBumper) {
            if (!clawSensor.getState()) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> barLeft.setPosition(0.469)),
                                new InstantCommand(() -> barRight.setPosition(1 - 0.469)),
                                new InstantCommand(() -> pivot.setPosition(0.625)),
                                new InstantCommand(() -> turret.setPosition(0.17))

                        )
                );
            } else {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> pivot.setPosition(0.545)),
                            new InstantCommand(() -> turret.setPosition(0.728)),
                            new InstantCommand(() -> barLeft.setPosition(0.06)),
                            new InstantCommand(() -> barRight.setPosition(1 - 0.06))
                    )
                );
            }
        }
        rightBumper2 = rightBumper;

        if (gamepad2.right_trigger > 0.3) {
            claw.setPosition(0.575);
            if (!clawSensor.getState()) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(84),
                                new InstantCommand(() -> barLeft.setPosition(0.385)),
                                new InstantCommand(() -> barRight.setPosition(1 - 0.385)),
                                new InstantCommand(() -> pivot.setPosition(0.545)),
                                new InstantCommand(() -> turret.setPosition(0.449))
                        )
                );
            }
        } else if (gamepad2.left_trigger > 0.3) {
            claw.setPosition(0.48);
        }

        boolean lt = gamepad1.right_trigger > 0.15;
        // CHECK IF IN LOW OR GROUND POSITION
        if ((!lastLT && lt) && !clawSensor.getState()) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> claw.setPosition(0.48)),
                            new WaitCommand(300),
                            new InstantCommand(() -> barLeft.setPosition(0.385)),
                            new InstantCommand(() -> barRight.setPosition(1 - 0.385)),
                            new InstantCommand(() -> pivot.setPosition(0.545)),
                            new InstantCommand(() -> turret.setPosition(0.728))
                    )
            );
        }
        lastLT = lt;

        double leftX = gamepad2.left_stick_x;
        if (Math.abs(leftX) > 0.4) {
            setFourbarFactor(joystickScalar(leftX, 0.4));
        }

        double rightX = gamepad2.right_stick_x;
        if (Math.abs(rightX) > 0.15) {
            setTurretFactor(joystickScalar(rightX, 0.15));
        }

        telemetry.addData("sensorState: ", !clawSensor.getState());
//        for (SwerveModule m : )
        telemetry.update();

        robot.drivetrain.set(drive);
        robot.drivetrain.updateModules();

        CommandScheduler.getInstance().run();

        robot.write();

        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return Math.signum(num) * min + (1 - min) * Math.pow(Math.abs(num), 3) * Math.signum(num);
    }

    public void setFourbarFactor(double factor) {
        double fourbarAddition = -0.001 * factor;
        double barLeftPos = barLeft.getPosition();
        if (!(barLeftPos + fourbarAddition > 0.469) || !(barLeftPos - fourbarAddition < 0.06)) {
            barLeft.setPosition(barLeftPos + fourbarAddition);
            barRight.setPosition(1 - barLeftPos + fourbarAddition);
        }
    }

    public void setTurretFactor(double factor) {
        double turretAddition = 0.007 * factor;
        double turretPos = turret.getPosition();
        turret.setPosition(turretPos + turretAddition);
    }
}
