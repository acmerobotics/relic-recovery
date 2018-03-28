package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Map;
import java.util.TreeMap;

public class SharpGP2Y0A51SK0FProximitySensor implements DistanceSensor {
    // the datasheet lists the range as 2 to 15 cm although there are values outside this range
    // TODO: decide whether the values outside are usable

    /* <volts, cm> */
    public static final TreeMap<Double, Double> VOLTAGE_TO_DISTANCE = new TreeMap<>();
    static {
        // earlier (out of range) values omitted
        VOLTAGE_TO_DISTANCE.put(2.45, 1.25);
        VOLTAGE_TO_DISTANCE.put(2.08, 2D);
        VOLTAGE_TO_DISTANCE.put(1.05, 5D);
        VOLTAGE_TO_DISTANCE.put(0.58, 10D);
        VOLTAGE_TO_DISTANCE.put(0.38, 15D);
        VOLTAGE_TO_DISTANCE.put(0.3, 20D);
    }

    private AnalogInput input;

    public SharpGP2Y0A51SK0FProximitySensor(AnalogInput analogInput) {
        input = analogInput;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        double voltage = input.getVoltage();
        Double distance = VOLTAGE_TO_DISTANCE.get(voltage);
        if (distance == null) {
            Map.Entry<Double, Double> floorEntry = VOLTAGE_TO_DISTANCE.floorEntry(voltage);
            Map.Entry<Double, Double> ceilEntry = VOLTAGE_TO_DISTANCE.ceilingEntry(voltage);

            if (floorEntry == null) {
                return ceilEntry.getValue();
            }

            if (ceilEntry == null) {
                return floorEntry.getValue();
            }

            double t = (voltage - floorEntry.getKey()) / (ceilEntry.getKey() - floorEntry.getKey());
            double interpolatedDistance = floorEntry.getValue() + t * (ceilEntry.getValue() - floorEntry.getValue());
            return unit.fromCm(interpolatedDistance);
        } else {
            return unit.fromCm(distance);
        }
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
