package com.acmerobotics.velocityvortex.opmodes;

import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.velocityvortex.sensors.TCS34725ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

/**
 * @author Ryan
 */

@TeleOp
public class TCS34725Test extends OpMode {

    private TCS34725ColorSensor colorSensor;
    private RobotDashboard dashboard;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(Arrays.asList(telemetry, dashboard.getTelemetry()));
        colorSensor = new TCS34725ColorSensor(hardwareMap.i2cDeviceSynch.get("color"), true);
        colorSensor.setIntegrationTime(TCS34725ColorSensor.IntegrationTime.INTEGRATION_TIME_24MS);
        colorSensor.enableLed(true);
        colorSensor.initialize();
    }

    @Override
    public void loop() {
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
    }
}
