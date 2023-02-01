package org.firstinspires.ftc.teamcode.common.hardware;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BetterDistanceSensor extends Rev2mDistanceSensorEx {
    private int requestRate;
    private boolean isActive;
    protected long time;
    private double distance;
    private DistanceUnit metric;
    private RANGING_PROFILE profile;

    public BetterDistanceSensor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, 100, DistanceUnit.CM, RANGING_PROFILE.HIGH_SPEED);
    }

    public BetterDistanceSensor(HardwareMap hardwareMap, String deviceName, int requestRate) {
        this(hardwareMap, deviceName, requestRate, DistanceUnit.CM, RANGING_PROFILE.HIGH_SPEED);
    }

    public BetterDistanceSensor(HardwareMap hardwareMap, String deviceName, int requestRate, DistanceUnit metric) {
        this(hardwareMap, deviceName, requestRate, metric, RANGING_PROFILE.HIGH_SPEED);
    }

    public BetterDistanceSensor(HardwareMap hardwareMap, String deviceName, int requestRate, DistanceUnit metric, RANGING_PROFILE profile) {
        super(hardwareMap.get(Rev2mDistanceSensor.class, deviceName).getDeviceClient());
        super.setRangingProfile(profile);
        this.requestRate = requestRate;
        this.isActive = true;
        this.time = System.currentTimeMillis();
        this.metric = metric;
        this.profile = profile;
    }

    public double request() {
        long current = System.currentTimeMillis();
        long deltaTime = current - time;
        if (deltaTime > requestRate && isActive) {
            this.distance = super.getDistance(metric);
            time = current;
        }

        return this.distance;
    }

    public void setRequestRate(int requestRate) {
        this.requestRate = requestRate;
    }

    public int getRequestRate() {
        return this.requestRate;
    }

    public void setDistanceUnit(DistanceUnit metric) {
        this.metric = metric;
    }

    public void setProfile(RANGING_PROFILE profile) {
        super.setRangingProfile(profile);
        this.profile = profile;
    }

    public void stop() {
        this.isActive = false;
    }

    public void start() {
        this.isActive = true;
    }

    public void setMode(boolean mode) {
        this.isActive = mode;
    }
}
