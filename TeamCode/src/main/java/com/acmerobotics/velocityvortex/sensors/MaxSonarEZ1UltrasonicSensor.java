package com.acmerobotics.velocityvortex.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class provides API access to the MaxSonar EZ1 Ultrasonic Sensor.
 */
public class MaxSonarEZ1UltrasonicSensor implements UltrasonicSensor, DistanceSensor {

    AnalogInput input;

    public MaxSonarEZ1UltrasonicSensor(AnalogInput analogInput) {
        input = analogInput;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return unit.fromInches(getUltrasonicLevel());
    }

    @Override
    public double getUltrasonicLevel() {
        return input.getVoltage() * 512.0 / input.getMaxVoltage();
    }

    @Override
    public String status() {
        return "";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MaxSonar EZ1 Ultrasonic Sensor";
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
