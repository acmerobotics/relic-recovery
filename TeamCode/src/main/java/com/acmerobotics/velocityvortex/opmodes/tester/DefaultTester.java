package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DefaultTester extends Tester<HardwareDevice> {
    public DefaultTester(String name, HardwareDevice device) {
        super(name, device);
    }

    @Override
    public String getType() {
        return device.getDeviceName();
    }

    @Override
    public String getId() {
        return "";
    }

    @Override
    public void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry) {
        telemetry.addData("conn_info", device.getConnectionInfo());
        telemetry.addData("version", device.getVersion());
        telemetry.addData("manufacturer", device.getManufacturer());
    }
}
