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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;

/**
 * @author Ryan
 */

@Config
@TeleOp(name = "Drive Velocity Characterization")
public class DriveVelocityCharacterization extends LinearOpMode {
    public static double AXIAL_POWER_INCREMENT = 0.05;
    public static double LATERAL_POWER_INCREMENT = 0.05;
    public static double HEADING_POWER_INCREMENT = 0.05;

    public static double LATERAL_DISTANCE = 48;
    public static double AXIAL_DISTANCE = 60;
    public static double MAX_HEADING_POWER = 0.9;

    public enum State {
        TUNE_LATERAL,
        TUNE_AXIAL,
        TUNE_HEADING,
        DONE
    }

    private RobotDashboard dashboard;
    private Looper looper;
    private MecanumDrive drive;
    private State state;
    private ElapsedTime timer;
    private Pose2d lastPose;

    @Override
    public void runOpMode() {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry());

        File logRoot = new File(LoggingUtil.getLogRoot(this), "DriveVelocityCharacterization-" + System.currentTimeMillis());
        logRoot.mkdirs();
        final CSVLoggingTelemetry axialLog = new CSVLoggingTelemetry(new File(logRoot, "Axial.csv"));
        final CSVLoggingTelemetry lateralLog = new CSVLoggingTelemetry(new File(logRoot, "Lateral.csv"));
        final CSVLoggingTelemetry headingLog = new CSVLoggingTelemetry(new File(logRoot, "Heading.csv"));

        state = State.TUNE_LATERAL;

        lastPose = new Pose2d(0, 0, 0);

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            telemetry.addData("state", state);

            double elapsedTime = timer.time();

            Pose2d estimatedPose = drive.getEstimatedPose();

            switch (state) {
                case TUNE_AXIAL: {
                    if (Math.abs(estimatedPose.x()) > AXIAL_DISTANCE) {
                        drive.stop();
                        reset();
                        state = State.TUNE_HEADING;
                    } else {
                        double power = AXIAL_POWER_INCREMENT * elapsedTime;
                        double axialDelta = estimatedPose.x() - lastPose.x();
                        double axialSpeed = axialDelta / dt;
                        drive.setVelocity(new Vector2d(power, 0), 0);

                        axialLog.addData("timestamp", timestamp);
                        axialLog.addData("power", power);
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
                        double power = LATERAL_POWER_INCREMENT * elapsedTime;
                        double lateralDelta = estimatedPose.y() - lastPose.y();
                        double lateralSpeed = lateralDelta / dt;
                        drive.setVelocity(new Vector2d(0, power), 0);

                        lateralLog.addData("timestamp", timestamp);
                        lateralLog.addData("power", power);
                        lateralLog.addData("speed (in/s)", lateralSpeed);
                        lateralLog.update();
                    }
                    break;
                }
                case TUNE_HEADING: {
                    double power = HEADING_POWER_INCREMENT * elapsedTime;
                    if (Math.abs(power) > MAX_HEADING_POWER) {
                        drive.stop();
                        reset();
                        state = State.DONE;
                    } else {
                        double headingDelta = estimatedPose.heading() - lastPose.heading();
                        if (Math.abs(headingDelta) > Math.PI / 2) {
                            headingDelta -= Math.signum(headingDelta) * 2 * Math.PI;
                        }
                        double omega = headingDelta / dt;
                        drive.setVelocity(new Vector2d(0, 0), power);

                        headingLog.addData("timestamp", timestamp);
                        headingLog.addData("power", power);
                        headingLog.addData("speed (rad/s)", omega);
                        headingLog.addData("heading", estimatedPose.heading());
                        headingLog.update();
                    }
                    break;
                }
            }

            lastPose = estimatedPose;

            telemetry.update();
        }));

        waitForStart();

        timer = new ElapsedTime();

        looper.start();

        while (opModeIsActive() && state != State.DONE) ;
    }

    private void reset() {
        timer.reset();
        lastPose = new Pose2d(0, 0, 0);
        drive.setEstimatedPose(new Pose2d(0, 0, 0));
        drive.setHeading(0);
    }
}
