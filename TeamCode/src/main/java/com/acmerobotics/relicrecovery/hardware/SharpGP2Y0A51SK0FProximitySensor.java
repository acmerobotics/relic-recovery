package com.acmerobotics.relicrecovery.hardware;

import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SharpGP2Y0A51SK0FProximitySensor implements HardwareDevice {
    // the datasheet lists the range as 2 to 15 cm although there are values outside this range
    // TODO: decide whether the values outside are usable
    public static final double A = 237.622, B = 9.5436, C = -2.3496, D = -12.2452;
    public static final double M_RED = 0.0265537, B_RED = 3.97514;
    public static final double M_BLUE = 0.123529, B_BLUE = 2.29412;

    private AnalogInput input;

    public SharpGP2Y0A51SK0FProximitySensor(AnalogInput analogInput) {
        input = analogInput;
    }

    public double getRawDistance() {
        double voltage = input.getVoltage();
        return A / (B * voltage + C) + D;
    }

    public double getDistance(AllianceColor color, DistanceUnit unit) {
        if (color == AllianceColor.RED) {
            return unit.fromCm(M_RED * getRawDistance() + B_RED);
        } else if (color == AllianceColor.BLUE) {
            return unit.fromCm(M_BLUE * getRawDistance() + B_BLUE);
        }
        return Double.NaN;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Sharp GP2Y0A51SK0F Proximity Sensor";
    }

    @Override
    public String getConnectionInfo() {
        return input.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        input.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        input.close();
    }
}
