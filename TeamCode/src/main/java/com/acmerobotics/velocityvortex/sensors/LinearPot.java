package com.acmerobotics.velocityvortex.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class provides API access to the LINEAR POTENTIOMETER
 */
public class LinearPot implements DistanceSensor {

    private AnalogInput input;
    private double length;

    public LinearPot(AnalogInput analogInput, double length, DistanceUnit unit) {
        input = analogInput;
        this.length = unit.toCm(length);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return unit.fromCm(input.getVoltage() / input.getMaxVoltage() * length);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Linear Potentiometer";
    }

    @Override
    public String getConnectionInfo() {
        return input.getConnectionInfo();
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
}
