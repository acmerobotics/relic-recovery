package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@Disabled
@Config
@TeleOp(name = "New Drive Characterization")
public class NewDriveCharacterization extends LinearOpMode {
    public static double LATERAL_DISTANCE = 72;
    public static double AXIAL_DISTANCE = 72;
    public static double HEADING_TIMEOUT = 5;

    public static double MAX_POWER = 0.85;

    public enum State {
        WAITING,
        TUNE_LATERAL,
        TUNE_AXIAL,
        TUNE_HEADING,
        DONE
    }

    private Robot robot;
    private State state = State.WAITING;
    private double startTimestamp;

    @Override
    public void runOpMode() {
        File logRoot = new File(LoggingUtil.getLogRoot(this), "NewDriveCharacterization-" + System.currentTimeMillis());
        logRoot.mkdirs();
        CSVLoggingTelemetry axialLog = new CSVLoggingTelemetry(new File(logRoot, "Axial.csv"));
        CSVLoggingTelemetry lateralLog = new CSVLoggingTelemetry(new File(logRoot, "Lateral.csv"));
        CSVLoggingTelemetry headingLog = new CSVLoggingTelemetry(new File(logRoot, "Heading.csv"));

        robot = new Robot(this);
        robot.drive.enablePositionEstimation();
        robot.addListener(() -> {
            telemetry.addData("state", state);

            double timestamp = TimestampedData.getCurrentTime();
            double elapsedTime = timestamp - startTimestamp;

            Pose2d estimatedPose = robot.drive.getEstimatedPose();

            switch (state) {
                case TUNE_AXIAL: {
                    if (Math.abs(estimatedPose.x()) > AXIAL_DISTANCE) {
                        reset();
                        state = State.TUNE_HEADING;
                    } else {
                        robot.drive.setVelocity(new Vector2d(MAX_POWER, 0), 0);
                        axialLog.addData("time", elapsedTime);
                        axialLog.addData("position", estimatedPose.x());
                        axialLog.update();
                    }
                    break;
                }
                case TUNE_LATERAL: {
                    if (Math.abs(estimatedPose.y()) > LATERAL_DISTANCE) {
                        reset();
                        state = State.TUNE_AXIAL;
                    } else {
                        robot.drive.setVelocity(new Vector2d(0, MAX_POWER), 0);
                        lateralLog.addData("time", elapsedTime);
                        lateralLog.addData("position", estimatedPose.y());
                        lateralLog.update();
                    }
                    break;
                }
                case TUNE_HEADING: {
                    if (elapsedTime > HEADING_TIMEOUT) {
                        reset();
                        state = State.DONE;
                    } else {
                        robot.drive.setVelocity(new Vector2d(0, 0), MAX_POWER);
                        headingLog.addData("time", elapsedTime);
                        headingLog.addData("position", estimatedPose.heading());
                        headingLog.update();
                    }
                    break;
                }
            }
        });
        robot.start();

        waitForStart();

        reset();

        state = State.TUNE_LATERAL;

        while (opModeIsActive() && state != State.DONE);
    }

    private void reset() {
        robot.drive.stop();
        robot.drive.update(null);
        sleep(1000);
        startTimestamp = TimestampedData.getCurrentTime();
        robot.drive.setEstimatedPose(new Pose2d(0, 0, 0));
    }
}
