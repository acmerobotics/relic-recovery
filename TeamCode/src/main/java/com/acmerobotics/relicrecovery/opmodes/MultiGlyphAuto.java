package com.acmerobotics.relicrecovery.opmodes;

import android.os.Looper;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.drive.CryptoboxLocalizer;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by ryanbrott on 12/4/17.
 */

@Config
@Autonomous(name = "Multi Glyph Auto")
public class MultiGlyphAuto extends LinearOpMode {
    public static boolean USE_VISION = true;
    public static boolean STRAFE_ALIGN = false;
    public static int VISION_UPDATE_MAX_DIST = -1;
    public static PIDCoefficients STRAFE_ALIGN_PID = new PIDCoefficients(-0.1, 0, 0);

    private PIDController strafeAlignController;

    private RobotDashboard dashboard;
    private Looper looper;
    private MecanumDrive drive;

    private VuforiaCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private CryptoboxLocalizer cryptoboxLocalizer;
    private FpsTracker fpsTracker;

    @Override
    public void runOpMode() throws InterruptedException {
        strafeAlignController = new PIDController(STRAFE_ALIGN_PID);

        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, telemetry);
        drive.setEstimatedPose(new Pose2d(48, -48, Math.PI));

        camera = new VuforiaCamera();
        cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        cryptoboxLocalizer = new CryptoboxLocalizer(cryptoboxTracker, camera.getProperties(), drive);
        cryptoboxTracker.disable();
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.initialize();

//        cryptoboxLocalizer.addListener((estimatedPos, timestamp) -> {
//                PositionEstimator positionEstimator = drive.getPositionEstimator();
//                if (!Double.isNaN(estimatedPos.x()) && !Double.isNaN(estimatedPos.y()) &&
//                        (VISION_UPDATE_MAX_DIST == -1 ||
//                                Vector2d.distance(positionEstimator.getPosition(), estimatedPos) < VISION_UPDATE_MAX_DIST)) {
//                    Log.i("CryptoTrackerListener", "vision update: " + estimatedPos);
//                    Log.i("CryptoTrackerListener", "old position: " + positionEstimator.getPosition());
//                    positionEstimator.setSlapperPosition(estimatedPos);
//                    Log.i("CryptoTrackerListener", "new position: " + positionEstimator.getPosition());
//                } else {
//                    Log.i("CryptoTrackerListener", "ignored vision update");
//                }
//        });

//        looper = new Looper();
//        drive.registerLoops(looper);
//        looper.addLoop(((timestamp, dt) -> {
//            dashboard.drawOverlay();
//            telemetry.update();
//        }));
//        looper.start();

        waitForStart();

        drive.followPath(new PathBuilder(new Pose2d(48, -48, Math.PI))
                .lineTo(new Vector2d(12, -48))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(12, -12))
                .build());
        waitForPathFollower();

        while (opModeIsActive()) {
            int choice = (int) (3 * Math.random());
            choice--;
            Vector2d v = new Vector2d(12 + choice * CryptoboxTracker.ACTUAL_RAIL_GAP, -60);

            if (USE_VISION) {
                cryptoboxTracker.enable();
            }

            sleep(1500);

            if (USE_VISION) {
                cryptoboxTracker.disable();
            }

            if (STRAFE_ALIGN) {
                drive.enableHeadingCorrection();
                drive.setTargetHeading(Math.PI / 2);

                strafeAlignController.reset();
                strafeAlignController.setSetpoint(v.x());
                while (opModeIsActive()) {
                    double error = strafeAlignController.getError(drive.getEstimatedPose().x());
                    if (Math.abs(error) < 1.5) {
                        drive.stop();
                        break;
                    }
                    drive.setVelocity(new Vector2d(0, strafeAlignController.update(error)), 0);
                    sleep(10);
                }

                drive.disableHeadingCorrection();

                drive.followPath(new PathBuilder(new Pose2d(v.x(), -12, Math.PI / 2)).lineTo(v).build());
                waitForPathFollower();
            } else {
                drive.followPath(new PathBuilder(new Pose2d(12, -12, Math.PI / 2)).lineTo(v).build());
                waitForPathFollower();
            }

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
