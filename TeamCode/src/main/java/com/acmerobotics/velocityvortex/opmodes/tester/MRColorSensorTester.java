package com.acmerobotics.velocityvortex.opmodes.tester;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by ACME Robotics on 2/26/2017.
 */

public class MRColorSensorTester extends ColorSensorTester<ModernRoboticsI2cColorSensor> {

    public MRColorSensorTester(String name, ModernRoboticsI2cColorSensor device) {
        super(name, device);

        device.setI2cAddress(I2cAddr.create8bit(0x3e));
        device.enableLed(false);
    }

    @Override
    public String getType() {
        return "MR Color Sensor";
    }

    @Override
    public String getId() {
        return "";
    }
}
