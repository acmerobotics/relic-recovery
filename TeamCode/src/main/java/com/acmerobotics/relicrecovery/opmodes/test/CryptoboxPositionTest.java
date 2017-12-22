package com.acmerobotics.relicrecovery.opmodes.test;

import android.util.Log;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.drive.PositionEstimator;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

/**
 * @author Ryan
 */

@TeleOp
public class CryptoboxPositionTest extends OpMode {
    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;
    private MecanumDrive drive;
    private Looper looper;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry(), new Pose2d(0, 0, Math.PI / 2));

        camera = new VisionCamera();
        cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE, drive);
        cryptoboxTracker.addListener(new CryptoboxTracker.CryptoboxTrackerListener() {
            @Override
            public void onCryptoboxDetection(List<Double> rails, Vector2d estimatedPos, double timestamp) {
                PositionEstimator positionEstimator = drive.getPositionEstimator();
                if (!Double.isNaN(estimatedPos.x()) && !Double.isNaN(estimatedPos.y()) ||
                        Vector2d.distance(positionEstimator.getPosition(), estimatedPos) < 6) {
                    Log.i("CryptoTrackerListener", "vision update: " + estimatedPos);
                    Log.i("CryptoTrackerListener", "old pose: " + positionEstimator.getPosition());
                    positionEstimator.setPosition(estimatedPos);
                    Log.i("CryptoTrackerListener", "new pose: " + positionEstimator.getPosition());
                } else {
                    Log.i("CryptoTrackerListener", "ignored vision update");
                }
            }
        });
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            Pose2d robotPose = drive.getEstimatedPose();
            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(robotPose.x(), robotPose.y(), 2);
            dashboard.drawOverlay();
            telemetry.addData("x", robotPose.x());
            telemetry.addData("y", robotPose.y());
            telemetry.update();
        }));
        looper.start();
    }

    @Override
    public void loop() {
        drive.setVelocity(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x);
    }

    @Override
    public void internalPostInitLoop() {

    }

    @Override
    public void internalPostLoop() {

    }

}
