package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.hardware.LynxEmbeddedIMUFactory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import java.io.File;

/**
 * Created by ryanbrott on 1/2/18.
 */

@TeleOp(name = "Expansion Hub Benchmark")
public class ExpansionHubBenchmark extends LinearOpMode {
    public static final int TRIALS = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        File logDir = new File(LoggingUtil.getLogRoot(this), "ExpansionHubBenchmark-" + System.currentTimeMillis());
        logDir.mkdirs();

        waitForStart();

        // I2C read test
        // internal IMU will be used for I2C
        BNO055IMU imu = LynxEmbeddedIMUFactory.createLynxEmbeddedIMU(
                hardwareMap.getAll(LynxModule.class).iterator().next());
        imu.initialize(new BNO055IMU.Parameters());

        CSVLoggingTelemetry i2cLogger = new CSVLoggingTelemetry(new File(logDir, "i2c.csv"));

        i2cLogger.addLine("I2C Read Benchmark Log");
        i2cLogger.addLine("Trials: " + TRIALS);

        MovingStatistics i2cStatistics = new MovingStatistics(TRIALS);
        for (int i = 0; i < TRIALS && opModeIsActive(); i++) {
            double startTime = TimestampedData.getCurrentTime();
            imu.getAngularOrientation();
            double dt = TimestampedData.getCurrentTime() - startTime;

            i2cStatistics.add(dt);

            i2cLogger.addData("trial", i);
            i2cLogger.addData("dt (sec)", dt);
            i2cLogger.update();
        }

        i2cLogger.addLine("Statistics");
        i2cLogger.addLine("Mean: " + i2cStatistics.getMean());
        i2cLogger.addLine("Standard Deviation: " + i2cStatistics.getStandardDeviation());

        // encoder read test
        // use generic motor encoder
        DcMotor motor = hardwareMap.dcMotor.iterator().next();

        CSVLoggingTelemetry encoderLogger = new CSVLoggingTelemetry(new File(logDir, "encoder.csv"));

        encoderLogger.addLine("Encoder Read Benchmark Log");
        encoderLogger.addLine("Trials: " + TRIALS);

        MovingStatistics encoderStatistics = new MovingStatistics(TRIALS);
        for (int i = 0; i < TRIALS && opModeIsActive(); i++) {
            double startTime = TimestampedData.getCurrentTime();
            motor.getCurrentPosition();
            double dt = TimestampedData.getCurrentTime() - startTime;

            encoderStatistics.add(dt);

            encoderLogger.addData("trial", i);
            encoderLogger.addData("dt (sec)", dt);
            encoderLogger.update();
        }

        encoderLogger.addLine("Statistics");
        encoderLogger.addLine("Mean: " + encoderStatistics.getMean());
        encoderLogger.addLine("Standard Deviation: " + encoderStatistics.getStandardDeviation());

        // set power
        CSVLoggingTelemetry powerLogger = new CSVLoggingTelemetry(new File(logDir, "power.csv"));

        powerLogger.addLine("Set Power Benchmark Log");
        powerLogger.addLine("Trials: " + TRIALS);

        MovingStatistics powerStatistics = new MovingStatistics(TRIALS);
        for (int i = 0; i < TRIALS && opModeIsActive(); i++) {
            double startTime = TimestampedData.getCurrentTime();
            motor.setPower(2 * Math.random() - 1);
            double dt = TimestampedData.getCurrentTime() - startTime;

            powerStatistics.add(dt);

            powerLogger.addData("trial", i);
            powerLogger.addData("dt (sec)", dt);
            powerLogger.update();
        }

        motor.setPower(0);

        powerLogger.addLine("Statistics");
        powerLogger.addLine("Mean: " + powerStatistics.getMean());
        powerLogger.addLine("Standard Deviation: " + powerStatistics.getStandardDeviation());

        telemetry.addLine("I2C Mean: " + i2cStatistics.getMean());
        telemetry.addLine("I2c Std Dev: " + i2cStatistics.getStandardDeviation());
        telemetry.addLine("Encoder Mean: " + encoderStatistics.getMean());
        telemetry.addLine("Encoder Std Dev: " + encoderStatistics.getStandardDeviation());
        telemetry.addLine("Power Mean: " + powerStatistics.getMean());
        telemetry.addLine("Power Sd Dev: " + powerStatistics.getStandardDeviation());
        telemetry.update();

        while (opModeIsActive());
    }
}
