package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

/**
 * Created by ryanbrott on 12/3/17.
 */

@Config
@TeleOp(name = "Axial Ramp Test")
public class AxialRampTest extends LinearOpMode {
    public static double DISTANCE = 4 * 24; // in
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
                new File(LoggingUtil.getLogRoot(this), "AxialRampData-" + System.currentTimeMillis() + ".csv"));

        looper = new Looper(100);
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            Pose2d estimatedPose = drive.getEstimatedPose();
            if (Math.abs(estimatedPose.x()) > DISTANCE) {
                drive.setVelocity(new Vector2d(0, 0), 0);
                running = false;
            } else if (startTimestamp == 0) {
                startTimestamp = timestamp;
            } else {
                double power = POWER_DERIVATIVE * (timestamp - startTimestamp) * 0.001;
                double axialDelta = estimatedPose.x() - lastPose.x();
                double axialSpeed = axialDelta / dt * 1000;
                drive.setVelocity(new Vector2d(power, 0), 0);

                loggingTelemetry.addData("timestamp", timestamp);
                loggingTelemetry.addData("power", power);
                loggingTelemetry.addData("speed (in/s)", axialSpeed);
                loggingTelemetry.update();
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
