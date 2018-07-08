package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.hardware.I2CXLMaxSonarEZ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestI2cUltrasonic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());

        I2CXLMaxSonarEZ sensor = new I2CXLMaxSonarEZ(hardwareMap.i2cDeviceSynch.get("ultra"));
        sensor.initialize();

        waitForStart();

        while (opModeIsActive()) {
            double distance = sensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("distance", distance);
            telemetry.update();
        }
    }
}
