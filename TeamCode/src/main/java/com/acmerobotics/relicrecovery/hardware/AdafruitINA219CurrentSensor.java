package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AdafruitINA219CurrentSensor implements CurrentSensor {
    private AnalogInput analogInput;

    public AdafruitINA219CurrentSensor(AnalogInput analogInput) {
        this.analogInput = analogInput;
    }

    @Override
    public double getCurrent() {
        return analogInput.getVoltage();
    }
}
