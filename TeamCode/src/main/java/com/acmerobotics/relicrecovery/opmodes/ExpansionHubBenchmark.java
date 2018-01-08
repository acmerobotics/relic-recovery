package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Iterator;

/**
 * Created by ryanbrott on 1/2/18.
 */

@TeleOp(name = "Expansion Hub Benchmark")
public class ExpansionHubBenchmark extends LinearOpMode {
    public static final int TRIALS = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule module = hardwareMap.getAll(LynxModule.class).iterator().next();

        telemetry.setAutoClear(false);
        telemetry.addLine("REV Expansion Hub Benchmark - " + module.getDeviceName());
        telemetry.addLine("Press start to begin...");
        telemetry.update();

        waitForStart();

        // IMU (unoptimized) test
        BNO055IMU imu = hardwareMap.getAll(BNO055IMU.class).iterator().next();
        imu.initialize(new BNO055IMU.Parameters());
        telemetry.addLine("IMU (unoptimized): " + formatResults(benchmarkOperation(imu::getAngularOrientation, TRIALS)));
        telemetry.update();
        imu.close();

        // IMU (optimized) test
        imu = LynxOptimizedI2cSensorFactory.createLynxEmbeddedIMU(module);
        imu.initialize(new BNO055IMU.Parameters());
        telemetry.addLine("IMU (optimized): " + formatResults(benchmarkOperation(imu::getAngularOrientation, TRIALS)));
        telemetry.update();
        imu.close();

        // Color (unoptimized test)
        LynxI2cColorRangeSensor colorRangeSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "colorRange");
        colorRangeSensor.initialize();
        telemetry.addLine("Color (unoptimized): " + formatResults(benchmarkOperation(colorRangeSensor::getNormalizedColors, TRIALS)));
        telemetry.update();
        colorRangeSensor.close();
        // Color (optimized test)
        // TODO have this automatically detect which bus the color range sensor is plugged into
        colorRangeSensor = LynxOptimizedI2cSensorFactory.createLynxI2cColorRangeSensor(module, 1);
        colorRangeSensor.initialize();
        telemetry.addLine("Color (optimized): " + formatResults(benchmarkOperation(colorRangeSensor::getNormalizedColors, TRIALS)));
        telemetry.update();
        colorRangeSensor.close();

        Iterator<DcMotor> dcMotorIterator = hardwareMap.getAll(DcMotor.class).iterator();
        if (dcMotorIterator.hasNext()) {
            // Encoder test
            DcMotor motor = hardwareMap.getAll(DcMotor.class).iterator().next();
            telemetry.addLine("Encoder read: " + formatResults(benchmarkOperation(motor::getCurrentPosition, TRIALS)));
            telemetry.update();
        } else {
            telemetry.addLine("Skipping encoder test - motor not found");
            telemetry.update();
        }

        telemetry.addLine("Done!");
        telemetry.update();

        while (opModeIsActive());
    }

    private static MovingStatistics benchmarkOperation(Func func, int trials) {
        MovingStatistics statistics = new MovingStatistics(trials);
        for (int i = 0; i < trials; i++) {
            double startTime = TimestampedData.getCurrentTime();
            func.value();
            double elapsedTime = TimestampedData.getCurrentTime() - startTime;
            statistics.add(elapsedTime);
        }
        return statistics;
    }

    private static String formatResults(MovingStatistics statistics) {
        return String.format("x\u0304 = %.2fms, \u03c3 = %.2fms", statistics.getMean() * 1000, statistics.getStandardDeviation() * 1000);
    }
}
