package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class provides API access to the MaxSonar EZ1 Ultrasonic Sensor.
 */
public class MaxSonarEZ1UltrasonicSensor implements UltrasonicSensor, DistanceSensor {
    public enum LogicLevel {
        V5,
        V3_3
    }

    private AnalogInput input;
    private LogicLevel logicLevel;

    public MaxSonarEZ1UltrasonicSensor(AnalogInput analogInput) {
        input = analogInput;
        logicLevel = input.getMaxVoltage() == 5 ? LogicLevel.V5 : LogicLevel.V3_3;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return unit.fromInches(getUltrasonicLevel());
    }

    public double getMinDistance(DistanceUnit unit) {
        return unit.fromInches(8);
    }

    @Override
    public double getUltrasonicLevel() {
        double voltage = input.getVoltage();
        if (logicLevel == LogicLevel.V5) {
            return voltage * 512.0 / input.getMaxVoltage();
        } else {
            return voltage * 1024.0 / input.getMaxVoltage();
        }
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
