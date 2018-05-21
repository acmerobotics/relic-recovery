package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.relicrecovery.localization.TrackingOmniLocalizer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.io.File;

@TeleOp
public class ProxLinearizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        robot.drive.enablePositionEstimation();
        robot.drive.setLocalizer(new TrackingOmniLocalizer(robot.drive));
        for (DcMotorEx motorEx : robot.drive.getMotors()) {
            motorEx.setMotorDisable();
        }

        CSVLoggingTelemetry logger = new CSVLoggingTelemetry(new File(
                LoggingUtil.getLogRoot(this), "ProxLinearizationTest-" + System.currentTimeMillis() + ".csv"));

        AnalogInput analogInput = hardwareMap.analogInput.get("proximitySensor");

        robot.addListener(() -> {
            logger.addData("proximityVoltage", analogInput.getVoltage());
            logger.addData("distance", robot.drive.getEstimatedPosition().y());
            logger.update();
        });

        waitForStart();

        robot.start();
        robot.drive.enableHeadingCorrection();
        robot.drive.setVelocity(new Vector2d(0, 0.02), 0);

        while (opModeIsActive());
    }
}
