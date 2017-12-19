package com.acmerobotics.relicrecovery.opmodes;

import android.util.Log;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.drive.PoseEstimator;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.LineSegment;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;
import java.util.List;

/**
 * Created by ryanbrott on 12/4/17.
 */

@Autonomous(name = "MultiGlyph Auto")
public class MultiGlyphAuto extends LinearOpMode {
    private Path pitToCrypto = new Path(Arrays.asList(
            new LineSegment(new Pose2d(12, -12, Math.PI / 2), new Pose2d(12, -60, Math.PI / 2))
    ));

    private Path cryptoToPit = new Path(Arrays.asList(
            new LineSegment(new Pose2d(12, -60, Math.PI / 2), new Pose2d(12, -12, Math.PI / 2))
    ));

    private RobotDashboard dashboard;
    private Looper looper;
    private MecanumDrive drive;

    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry(), new Pose2d(48, -48, Math.PI));

        camera = new VisionCamera();
        cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.setImageDir(LoggingUtil.getImageDir(this));
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        cryptoboxTracker.addListener(new CryptoboxTracker.CryptoboxTrackerListener() {
            @Override
            public void onCryptoboxDetection(List<Double> rails, Vector2d estimatedPos, double timestamp) {
                PoseEstimator poseEstimator = drive.getPoseEstimator();
                Log.i("CryptoTrackerListener", "vision update: " + estimatedPos);
                Log.i("CryptoTrackerListener", "old pose: " + poseEstimator.getPose());
                poseEstimator.updatePose(new Pose2d(estimatedPos, 0), timestamp);
                poseEstimator.setPose(new Pose2d(poseEstimator.getPose().pos(), drive.getHeading()));
                Log.i("CryptoTrackerListener", "new pose: " + poseEstimator.getPose());
            }
        });

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            dashboard.drawOverlay();
            telemetry.update();
        }));
        looper.start();

        waitForStart();

        drive.followPath(new PathBuilder(new Pose2d(48, -48, Math.PI))
                .lineTo(new Vector2d(12, -48))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(12, -12))
                .build());
        waitForPathFollower();

        for (int i = 0; i < 10 && opModeIsActive(); i++) {
            int choice = (int) (3 * Math.random());
            choice--;
            Vector2d v = new Vector2d(12 + choice * CryptoboxTracker.ACTUAL_RAIL_GAP, -60);

            sleep(1500);
            drive.followPath(new PathBuilder(new Pose2d(12, -12, Math.PI / 2)).lineTo(v).build());
            waitForPathFollower();

            sleep(1500);
            drive.followPath(new PathBuilder(new Pose2d(v, Math.PI / 2)).lineTo(new Vector2d(12, -12)).build());
            waitForPathFollower();
        }
    }

    private void waitForPathFollower() {
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }
    }
}
