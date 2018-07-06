package com.acmerobotics.library.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

/**
 * Interface for the I2CXL MaxSonar EZ ultrasonic sensor from MaxBotix. Based on an earlier version
 * by 4634 FROGbots
 */
@I2cSensor(name = "I2CXL MaxSonar EZ", description = "Ultrasonic Sensor from MaxBotix", xmlTag = "I2CXLMaxSonarEZ")
public class I2CXLMaxSonarEZ extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, I2CXLMaxSonarEZ.Parameters> {
    public static class Parameters {
        public I2cAddr i2cAddr = I2CADDR_DEFAULT;
        public long msRangeWait = 100; // 100ms is the minimum recommended by the data sheet
    }

    public static final I2cAddr I2CADDR_DEFAULT = I2cAddr.create8bit(0xE0);

    public I2CXLMaxSonarEZ(I2cDeviceSynch deviceClient) {
        super(deviceClient, true, new Parameters());

        this.deviceClient.engage();

        this.registerArmingStateCallback(false);
    }

    @Override
    protected boolean internalInitialize(@NonNull Parameters parameters) {
        this.deviceClient.setI2cAddress(parameters.i2cAddr);

        return true;
    }

    public double getDistance(DistanceUnit unit) {
        deviceClient.write8(0, 0x51, I2cWaitControl.WRITTEN);

        try {
            Thread.sleep(parameters.msRangeWait);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        double rawDistance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));
        return unit.fromCm(rawDistance);
    }

    public double getAverageDistance(int samples, DistanceUnit unit) {
        double sum = 0;
        for (int i = 0; i < samples; i++) {
            sum += getDistance(unit);
        }
        return sum / samples;
    }

    @Override
    public String getDeviceName() {
        return "I2CXLMaxSonarEZ";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
}