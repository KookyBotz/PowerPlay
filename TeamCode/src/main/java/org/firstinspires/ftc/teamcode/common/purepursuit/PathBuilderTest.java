package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.controller.PurePursuitPathBuilder;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.purepursuit.drive.Localizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

@TeleOp
@Config
public class PathBuilderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveChassis swerve = new SwerveChassis(hardwareMap);
        Drivetrain drivetrain = swerve.drivetrain;
        Localizer localizer = new BetterSwerveLocalizer(() -> -swerve.getAngle(), swerve.drivetrain.modules);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
        waitForStart();
        swerve.startIMUThread(this);

        PurePursuitPath path = new PurePursuitPathBuilder()
                .setDrivetrain(drivetrain)
                .setLocalizer(localizer)
                .setPower(1)
                .then(new Pose())
                .then(new Pose(0, 40, 0))
                .setPower(0.5)
                .then(new Pose(40, 40, 0))
                .build();

        while (opModeIsActive()) {
            localizer.periodic();
            path.update();
            swerve.drivetrain.updateModules();

            telemetry.addData("pose", localizer.getPos());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}
