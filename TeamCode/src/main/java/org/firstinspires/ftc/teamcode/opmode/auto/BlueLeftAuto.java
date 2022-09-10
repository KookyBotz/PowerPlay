package org.firstinspires.ftc.teamcode.opmode.auto;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.BetterSwerveLocalizer;
import org.firstinspires.ftc.teamcode.common.purepursuit.localizer.Localizer;

public class BlueLeftAuto extends LinearOpMode {


    PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    PhotonCore.enable();

    @Override
    public void runOpMode() throws InterruptedException {
        Localizer localizer = new BetterSwerveLocalizer(() -> swerve.getAngle(), swerve.drivetrain.modules);
    }
}
