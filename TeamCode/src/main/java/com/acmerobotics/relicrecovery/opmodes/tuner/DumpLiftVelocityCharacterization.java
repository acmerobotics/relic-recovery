package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp
public class DumpLiftVelocityCharacterization extends LinearOpMode {
    public static final double POWER_INCREMENT = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());
        DumpBed dumpBed = new DumpBed(hardwareMap, telemetry);

        File logFile = new File(LoggingUtil.getLogRoot(this), "DumpLiftVelocityCharacterization-" + System.currentTimeMillis() + ".csv");
        CSVLoggingTelemetry logger = new CSVLoggingTelemetry(logFile);

        waitForStart();

        dumpBed.calibrate();
        while (!dumpBed.isCalibrated()) {
            dumpBed.update();
            telemetry.update();
        }

        double startTimestamp = TimestampedData.getCurrentTime();
        double lastTimestamp = startTimestamp;
        double lastLiftHeight = 0;

        while (opModeIsActive()) {
            double timestamp = TimestampedData.getCurrentTime();
            double dt = timestamp - lastTimestamp;
            double elapsedTime = timestamp - startTimestamp;

            double power = POWER_INCREMENT * elapsedTime;
            dumpBed.setLiftPower(power);

            double liftHeight = dumpBed.getLiftHeight();
            if (liftHeight >= 0.75 * DumpBed.LIFT_HEIGHT) {
                dumpBed.setLiftPower(0);
                break;
            }
            double heightDelta = liftHeight - lastLiftHeight;
            double speed = heightDelta / dt;

            logger.addData("power", power);
            logger.addData("speed", speed);
            logger.update();

            lastLiftHeight = liftHeight;
            lastTimestamp = timestamp;
        }

        telemetry.clearAll();
        telemetry.log().add("Finished characterization");
        telemetry.update();

        while (opModeIsActive());
    }
}
