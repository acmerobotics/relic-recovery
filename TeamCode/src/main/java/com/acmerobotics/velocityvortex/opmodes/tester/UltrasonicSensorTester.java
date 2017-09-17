package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UltrasonicSensorTester extends Tester<UltrasonicSensor> {
    public UltrasonicSensorTester(String name, UltrasonicSensor device) {
        super(name, device);
    }

    @Override
    public String getType() {
        return "Ultrasonic Sensor";
    }

    @Override
    public String getId() {
        return "";
    }

    @Override
    public void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry) {
        telemetry.addData("ultrasonic_level", device.getUltrasonicLevel());
    }
}
