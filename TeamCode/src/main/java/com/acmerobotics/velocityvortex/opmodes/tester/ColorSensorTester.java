package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class ColorSensorTester<T extends ColorSensor> extends Tester<T> {
    public ColorSensorTester(String name, T device) {
        super(name, device);
    }

    @Override
    public void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry) {
        telemetry.addData("alpha", device.alpha());
        telemetry.addData("red", device.red());
        telemetry.addData("green", device.green());
        telemetry.addData("blue", device.blue());
    }
}
