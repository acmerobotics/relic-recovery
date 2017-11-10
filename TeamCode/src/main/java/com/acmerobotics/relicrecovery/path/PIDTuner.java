package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 11/9/17.
 */

@Config
@TeleOp
public class PIDTuner extends OpMode {
    public static final double RADIUS = 10;
    
    public static Pose2d setpoint;

    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    private Looper looper;

    private MecanumDrive drive;
    private PIDController headingController, axialController, lateralController;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        fieldOverlay = dashboard.getFieldOverlay();

        drive = new MecanumDrive(hardwareMap);

        headingController = new PIDController(DriveConstants.HEADING_COEFFS);
        axialController = new PIDController(DriveConstants.AXIAL_COEFFS);
        lateralController = new PIDController(DriveConstants.LATERAL_COEFFS);

        looper = new Looper(20);
        looper.addLoop((timestamp, dt) -> {
            double time = getRuntime();

            Pose2d robotPose = drive.getEstimatedPose();

            headingController.setSetpoint(setpoint.heading());
            double headingError = headingController.getError(robotPose.heading());
            double headingUpdate = headingController.update(headingError, time);

            Vector2d fieldError = robotPose.pos().add(setpoint.pos().negated());
            Vector2d robotError = fieldError.rotated(robotPose.heading());

            double lateralError = robotError.x();
            double axialError = robotError.y();

            axialController.setSetpoint(setpoint.x());
            double axialUpdate = axialController.update(axialError, time);

            lateralController.setSetpoint(setpoint.y());
            double lateralUpdate = lateralController.update(lateralError, time);

            drive.setVelocity(new Vector2d(axialUpdate, lateralUpdate), headingUpdate);

            telemetry.addData("x", robotPose.x());
            telemetry.addData("y", robotPose.y());
            telemetry.addData("heading", robotPose.heading());

            telemetry.addData("headingError", headingError);
            telemetry.addData("headingUpdate", headingUpdate);

            telemetry.addData("axialError", axialError);
            telemetry.addData("axialUpdate", axialUpdate);

            telemetry.addData("lateralError", lateralError);
            telemetry.addData("lateralUpdate", lateralUpdate);

            fieldOverlay.setFill("blue");
            fieldOverlay.setStrokeWidth(2);
            fieldOverlay.strokeLine(
                    robotPose.x() + 0.5 * RADIUS * Math.cos(robotPose.heading()),
                    robotPose.y() + 0.5 * RADIUS * Math.sin(robotPose.heading()),
                    robotPose.x() + RADIUS * Math.cos(robotPose.heading()),
                    robotPose.y() + RADIUS * Math.sin(robotPose.heading()));
            fieldOverlay.strokeCircle(robotPose.x(), robotPose.y(), RADIUS);
            dashboard.drawOverlay();
        });

        drive.registerLoops(looper);

        looper.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        looper.terminate();
    }
}
