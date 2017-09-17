package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorTester extends Tester<DistanceSensor> {
    public DistanceSensorTester(String name, DistanceSensor device) {
        super(name, device);
    }

    @Override
    public String getType() {
        return "Distance Sensor";
    }

    @Override
    public String getId() {
        return "";
    }

    @Override
    public void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry) {
        telemetry.addData("distance", device.getDistance(DistanceUnit.INCH) + "in");
    }
}
