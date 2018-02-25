package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@Config
@TeleOp(name = "New Drive Characterization")
public class NewDriveCharacterization extends LinearOpMode {
    public static double LATERAL_DISTANCE = 60;
    public static double AXIAL_DISTANCE = 60;
    public static double HEADING_TIMEOUT = 10;

    public static double MAX_POWER = 0.85;

    public enum State {
        TUNE_LATERAL,
        TUNE_AXIAL,
        TUNE_HEADING,
        DONE
    }

    private MecanumDrive drive;
    private double startTimestamp;
    private Pose2d lastPose;

    private double lastTimestamp;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, telemetry);
        drive.enablePositionEstimation();

        File logRoot = new File(LoggingUtil.getLogRoot(this), "NewDriveCharacterization-" + System.currentTimeMillis());
        logRoot.mkdirs();
        CSVLoggingTelemetry axialLog = new CSVLoggingTelemetry(new File(logRoot, "Axial.csv"));
        CSVLoggingTelemetry lateralLog = new CSVLoggingTelemetry(new File(logRoot, "Lateral.csv"));
        CSVLoggingTelemetry headingLog = new CSVLoggingTelemetry(new File(logRoot, "Heading.csv"));

        State state = State.TUNE_LATERAL;

        waitForStart();

        reset();

        while (opModeIsActive() && state != State.DONE) {
            telemetry.addData("state", state);

            double timestamp = TimestampedData.getCurrentTime() - startTimestamp;
            double elapsedTime = timestamp - startTimestamp;
            double dt = timestamp - lastTimestamp;
            lastTimestamp = timestamp;

            Pose2d estimatedPose = drive.getEstimatedPose();

            switch (state) {
                case TUNE_AXIAL: {
                    if (Math.abs(estimatedPose.x()) > AXIAL_DISTANCE) {
                        drive.stop();
                        reset();
                        state = State.TUNE_HEADING;
                    } else {
                        double axialDelta = estimatedPose.x() - lastPose.x();
                        double axialSpeed = axialDelta / dt;
                        drive.setVelocity(new Vector2d(MAX_POWER, 0), 0);

                        axialLog.addData("timestamp", timestamp);
                        axialLog.addData("speed (in/s)", axialSpeed);
                        axialLog.update();
                    }
                    break;
                }
                case TUNE_LATERAL: {
                    if (Math.abs(estimatedPose.y()) > LATERAL_DISTANCE) {
                        drive.stop();
                        reset();
                        state = State.TUNE_AXIAL;
                    } else {
                        double lateralDelta = estimatedPose.y() - lastPose.y();
                        double lateralSpeed = lateralDelta / dt;
                        drive.setVelocity(new Vector2d(0, MAX_POWER), 0);

                        lateralLog.addData("timestamp", timestamp);
                        lateralLog.addData("speed (in/s)", lateralSpeed);
                        lateralLog.update();
                    }
                    break;
                }
                case TUNE_HEADING: {
                    if (elapsedTime > HEADING_TIMEOUT) {
                        drive.stop();
                        reset();
                        state = State.DONE;
                    } else {
                        double headingDelta = estimatedPose.heading() - lastPose.heading();
                        if (Math.abs(headingDelta) > Math.PI / 2) {
                            headingDelta -= Math.signum(headingDelta) * 2 * Math.PI;
                        }
                        double omega = headingDelta / dt;
                        drive.setVelocity(new Vector2d(0, 0), MAX_POWER);

                        headingLog.addData("timestamp", timestamp);
                        headingLog.addData("speed (rad/s)", omega);
                        headingLog.update();
                    }
                    break;
                }
            }

            lastPose = estimatedPose;

            drive.update();
            telemetry.update();
        }
    }

    private void reset() {
        startTimestamp = TimestampedData.getCurrentTime();
        lastPose = new Pose2d(0, 0, 0);
        drive.setEstimatedPose(new Pose2d(0, 0, 0));
        lastTimestamp = 0;
    }
}
