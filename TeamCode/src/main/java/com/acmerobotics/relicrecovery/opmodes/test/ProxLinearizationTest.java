package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.library.util.TimestampedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.io.File;

@TeleOp
public class ProxLinearizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput proximitySensor = hardwareMap.analogInput.get("prox");
        AnalogInput potentiometer = hardwareMap.analogInput.get("pot");

        CSVLoggingTelemetry logger = new CSVLoggingTelemetry(new File(
                LoggingUtil.getLogRoot(this), "ProxLinearizationTest-" + System.currentTimeMillis() + ".csv"));

        waitForStart();

        while (!isStopRequested()) {
            logger.addData("timestamp", TimestampedData.getCurrentTime());
            logger.addData("proxVoltage", proximitySensor.getVoltage());
            logger.addData("potVoltage", potentiometer.getVoltage());
            logger.update();
        }
    }
}
