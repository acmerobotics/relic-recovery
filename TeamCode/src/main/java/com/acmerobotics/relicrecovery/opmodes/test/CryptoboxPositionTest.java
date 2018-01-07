package com.acmerobotics.relicrecovery.opmodes.test;

import android.util.Log;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.drive.CryptoboxLocalizer;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.drive.PositionEstimator;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Ryan
 */

@TeleOp
public class CryptoboxPositionTest extends OpMode {
    private VuforiaCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private CryptoboxLocalizer cryptoboxLocalizer;
    private FpsTracker fpsTracker;
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;
    private MecanumDrive drive;
    private Looper looper;
    private PriorityScheduler scheduler;

    @Override
    public void init() {
        scheduler = new PriorityScheduler();
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        drive = new MecanumDrive(hardwareMap, scheduler, dashboard.getTelemetry());
        drive.setEstimatedPose(new Pose2d(0, 0, Math.PI / 2));

        camera = new VuforiaCamera();
        cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        cryptoboxLocalizer = new CryptoboxLocalizer(cryptoboxTracker, camera.getProperties(), drive);
        cryptoboxLocalizer.addListener((estimatedPos, timestamp) -> {
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
        });
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.initialize();

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            Cryptobox closestCryptobox = cryptoboxLocalizer.getClosestCryptobox();
            Vector2d cryptoboxPos = closestCryptobox.getPose().pos();
            fieldOverlay.setFill("goldenrod");
            fieldOverlay.fillCircle(cryptoboxPos.x(), cryptoboxPos.y(), 1);
            dashboard.drawOverlay();
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
