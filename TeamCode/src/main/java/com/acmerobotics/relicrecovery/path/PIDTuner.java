package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
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
    public static final double RADIUS = 9 * Math.sqrt(2);
    
    public static double xSetpoint = 0;
    public static double ySetpoint = 0;
    public static double headingSetpoint = 0;

    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    private Looper looper;

    private MecanumDrive drive;
    private PIDController headingController, axialController, lateralController;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();

        String opModeDir = "PIDTuner-" + System.currentTimeMillis();
//        drive = new MecanumDrive(hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(),
//                new CSVLoggingTelemetry(new File(DataFile.getStorageDir(),
//                        opModeDir + File.pathSeparator + "MecanumDrive.csv").getPath())));
        drive = new MecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        headingController = new PIDController(DriveConstants.HEADING_COEFFS);
        axialController = new PIDController(DriveConstants.AXIAL_COEFFS);
        lateralController = new PIDController(DriveConstants.LATERAL_COEFFS);

        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setOutputBounds(-1, 1);

        axialController.setOutputBounds(-1, 1);

        lateralController.setOutputBounds(-1, 1);

        looper = new Looper();
        looper.addLoop((timestamp, dt) -> {
            double time = getRuntime();

            Pose2d robotPose = drive.getEstimatedPose();

            headingController.setSetpoint(headingSetpoint);
            double headingError = headingController.getError(robotPose.heading());
            double headingUpdate = headingController.update(headingError, time);

            Pose2d pose = new Pose2d(xSetpoint, ySetpoint, headingSetpoint);
            Vector2d fieldError = robotPose.pos().added(pose.pos().negated());
            Vector2d robotError = fieldError.rotated(-robotPose.heading());

            double axialError = robotError.x();
            double lateralError = robotError.y();

            double axialUpdate = axialController.update(axialError, time);
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

            telemetry.update();

            fieldOverlay.setFill("green");
            fieldOverlay.fillCircle(xSetpoint, ySetpoint, 1);

            fieldOverlay.setFill("blue");
            fieldOverlay.setStrokeWidth(4);
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
    public void internalPostInitLoop() {

    }

    @Override
    public void internalPostLoop() {

    }

    @Override
    public void loop() {

    }
}
