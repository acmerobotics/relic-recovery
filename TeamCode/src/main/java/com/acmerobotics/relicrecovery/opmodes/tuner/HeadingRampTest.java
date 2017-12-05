package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

/**
 * Created by ryanbrott on 12/3/17.
 */

@Config
@TeleOp(name = "Heading Ramp Test")
public class HeadingRampTest extends LinearOpMode {
    public static double MAX_POWER = 0.6; // power
    public static double POWER_DERIVATIVE = 0.05; // power units/s

    private Looper looper;
    private MecanumDrive drive;
    private long startTimestamp;
    private Pose2d lastPose;
    private volatile boolean running;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry);
        drive = new MecanumDrive(hardwareMap);
        final CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(
                new File(LoggingUtil.getLogRoot(this), "HeadingRampData-" + System.currentTimeMillis() + ".csv"));

        looper = new Looper(100);
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            Pose2d estimatedPose = drive.getEstimatedPose();
            if (startTimestamp == 0) {
                startTimestamp = timestamp;
            } else {
                double power = POWER_DERIVATIVE * (timestamp - startTimestamp) * 0.001;
                if (Math.abs(power) > MAX_POWER) {
                    drive.setVelocity(new Vector2d(0, 0), 0);
                    running = false;
                } else {
                    double headingDelta = estimatedPose.heading() - lastPose.heading();
                    double omega = headingDelta / dt * 1000;
                    drive.setVelocity(new Vector2d(0, 0), power);

                    loggingTelemetry.addData("timestamp", timestamp);
                    loggingTelemetry.addData("power", power);
                    loggingTelemetry.addData("omega (rad/s)", omega);
                    loggingTelemetry.update();
                }
            }

            lastPose = estimatedPose;

            telemetry.update();
        }));

        waitForStart();

        running = true;
        looper.start();

        while (opModeIsActive() && running);
    }
}
